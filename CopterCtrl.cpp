#include "CopterCtrl.hpp"
#include "accelerometer.hpp"
#include "gyro.hpp"
#include "flightcontrol.hpp"

#include <QTimer>
#include <QTcpSocket>

#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

CopterCtrl::CopterCtrl() :
  m_tcpServer(),
  m_tcpConnection(),
  m_debugTcpServer(),
  m_debugTcpConnection()
{
	initSettings();
	
	m_flightControl = QSharedPointer<FlightControl>(new FlightControl(this));
	
	// tcp server setup
	m_tcpServer.listen(QHostAddress::Any, m_settings->value("TcpPort").toInt());
	connect(&m_tcpServer, SIGNAL(newConnection()), this, SLOT(onTcpConnection()));
	
	// debug tcp server setup
	m_debugTcpServer.listen(QHostAddress::Any, m_settings->value("DebugPort").toInt());
	connect(&m_debugTcpServer, SIGNAL(newConnection()), this, SLOT(onDebugTcpConnection()));
	
	// buttons reading
	const QString s_buttons_input_path = m_settings->value("ButtonsInputPath").toString();
	m_buttonsInputFd = ::open(s_buttons_input_path.toLatin1().data(), O_SYNC, O_RDONLY);
	if (m_buttonsInputFd == -1)
		qDebug() << "Cannot open buttons input file " << s_buttons_input_path << ", reason: " << errno;
	m_buttonsInputNotifier = new QSocketNotifier(m_buttonsInputFd, QSocketNotifier::Read, this);
	connect(m_buttonsInputNotifier, SIGNAL(activated(int)), this, SLOT(onButtonRead()));
	m_buttonsInputNotifier->setEnabled(true);
}

CopterCtrl::~CopterCtrl() { }

void CopterCtrl::initSettings()
{
	m_settings = QSharedPointer<QSettings>(new QSettings(QApplication::applicationDirPath() + "/config.ini", 
	                                                     QSettings::IniFormat));
	
	// TODO: write proper checker
	if (m_settings->allKeys().count() == 0) {
		// TODO: move to conf file
		m_settings->setValue("AccelInputPath", "/dev/input/event1");
		m_settings->setValue("AccelHistoryLength", 5);
		m_settings->setValue("GyroInputPath", "/dev/input/event2");
		m_settings->setValue("ButtonsInputPath", "/dev/input/event0");
		m_settings->setValue("TcpPort", 4000);
		m_settings->setValue("DebugPort", 7777);
		m_settings->setValue("PowerStep1", 1);
		m_settings->setValue("PowerStep2", 5);
		m_settings->setValue("PowerMin", 0);
		m_settings->setValue("PowerMax", 100);
		m_settings->setValue("MotorMax", 1740000);
		m_settings->setValue("MotorMin", 1200000);
		m_settings->setValue("KalmanK", 0.95);
		m_settings->setValue("PidP", -0.02d);
		m_settings->setValue("PidI", 0.0);
		m_settings->setValue("PidD", -0.005d);
		m_settings->setValue("PidIWindow", 10);
		m_settings->setValue("FilterMethod", 3);
		m_settings->setValue("MotorPathX1", "/sys/devices/platform/ehrpwm.0/pwm/ehrpwm.0:0/duty_ns");
		m_settings->setValue("MotorPathX2", "/sys/devices/platform/ehrpwm.0/pwm/ehrpwm.0:1/duty_ns");
		m_settings->setValue("MotorPathY1", "/sys/devices/platform/ehrpwm.1/pwm/ehrpwm.1:1/duty_ns");
		m_settings->setValue("MotorPathY2", "/sys/devices/platform/ehrpwm.1/pwm/ehrpwm.1:0/duty_ns");
		m_settings->setValue("NoGraphics", true);
		m_settings->setValue("MotorIntervalAlpha", 0.5);
		m_settings->setValue("DerivativeK", 0.6);
		m_settings->setValue("GyroMappingCoeff", 938);
		m_settings->setValue("AccelCorrectingCoeff", 0.01);
	}
	
	m_settings->setFallbacksEnabled(false);
	m_settings->sync();
	
	connect(this, SIGNAL(settingsValueChanged(QString,QVariant)), this, SLOT(onSettingsValueChange(QString,QVariant)));
}

void CopterCtrl::adjustSettingsValue(const QString &key, QMetaType::Type type, bool increase)
{
	switch(type) {
		case QMetaType::Int:
			m_settings->setValue(key, m_settings->value(key).toInt() + (increase ? 1 : -1));
			break;
		case QMetaType::Float:
			m_settings->setValue(key, m_settings->value(key).toFloat() * (increase ? (1.0 / 0.9) : 0.9));
			break;
		case QMetaType::Double:
			m_settings->setValue(key, m_settings->value(key).toDouble() * (increase ? (1.0 / 0.9) : 0.9));
			break;
		case QMetaType::Bool:
			m_settings->setValue(key, increase);
			break;
		default:
			tcpLog("Inappropriate type for adjusting: " + QString(QMetaType::typeName(type)));
			return;
			break;
	}
	emit settingsValueChanged(key, m_settings->value(key));
}

void CopterCtrl::onSettingsValueChange(const QString &key, const QVariant &value)
{
	if (value.canConvert(QVariant::String)) {
		tcpLog("Settings value for key " + key + " changed. New value: " + value.toString());
	}
	else {
		tcpLog("Settings value for key " + key + " changed. Value is inconvertable to string");
	}
}

void CopterCtrl::emergencyStop() {
	m_flightControl->emergencyStop();
}

void CopterCtrl::tcpLog(const QString &message)
{
	if (!m_tcpConnection.isNull() && m_tcpConnection->isValid()) {
		m_tcpConnection->write(message.toAscii());
		m_tcpConnection->write("\r\n");
	}
}

void CopterCtrl::onTcpConnection()
{
	if (!m_tcpConnection.isNull())
		qDebug() << "Replacing existing connection";
	m_tcpConnection = m_tcpServer.nextPendingConnection();
	qDebug() << "Accepted new connection";
	m_tcpConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
	connect(m_tcpConnection, SIGNAL(disconnected()), this, SLOT(onTcpDisconnection()));
	connect(m_tcpConnection, SIGNAL(readyRead()), this, SLOT(onTcpNetworkRead()));
}

void CopterCtrl::onTcpDisconnection()
{
	qDebug() << "Existing connection disconnected";
	m_tcpConnection->deleteLater();
	m_tcpConnection = NULL;
}

void CopterCtrl::onTcpNetworkRead()
{
	if (m_tcpConnection.isNull())
		return;
	
	static const int s_power_max = m_settings->value("PowerMax").toInt();
	//static const int s_power_min = m_settings->value("PowerMin").toInt();
	static const int s_power_step1 = m_settings->value("PowerStep1").toInt();
	static const int s_power_step2 = m_settings->value("PowerStep2").toInt();
	
	while (m_tcpConnection->isReadable())
	{
		char c;
		if (!m_tcpConnection->getChar(&c))
			break;
		switch (c)
		{
			case 'Z': m_flightControl->adjustPower(-s_power_max); break;
			case 'z': m_flightControl->adjustPower(-s_power_step2); break;
			case 'x': m_flightControl->adjustPower(-s_power_step1); break;
			case 'c': m_flightControl->adjustPower(+s_power_step1); break;
			case 'v': m_flightControl->adjustPower(+s_power_step2); break;
			case 'V': m_flightControl->adjustPower(+s_power_max); break;
			case '0': m_flightControl->setupAccelZeroAxis(); break;
			case 'a': emergencyStop(); break;
			case '[': adjustSettingsValue("MotorMax", QMetaType::Int, false); break;
			case ']': adjustSettingsValue("MotorMax", QMetaType::Int, true); break;
			case '{': adjustSettingsValue("MotorMin", QMetaType::Int, false); break;
			case '}': adjustSettingsValue("MotorMin", QMetaType::Int, true); break;
			case ',': adjustSettingsValue("PidP", QMetaType::Float, false); break;
			case '.': adjustSettingsValue("PidP", QMetaType::Float, true); break;
			case '<': adjustSettingsValue("PidD", QMetaType::Float, false); break;
			case '>': adjustSettingsValue("PidD", QMetaType::Float, true); break;
			case '(': adjustSettingsValue("PidI", QMetaType::Float, false); break;
			case ')': adjustSettingsValue("PidI", QMetaType::Float, true); break;
		}
	}
}

void CopterCtrl::debugTcpLog(const QString &message)
{
	if (!m_debugTcpConnection.isNull() && m_debugTcpConnection->isValid()) {
		m_debugTcpConnection->write(message.toAscii());
		m_debugTcpConnection->write("\r\n");
	}
}

void CopterCtrl::onDebugTcpConnection()
{
	if (!m_debugTcpConnection.isNull())
		qDebug() << "Replacing existing debug connection";
	m_debugTcpConnection = m_debugTcpServer.nextPendingConnection();
	qDebug() << "Accepted new debug connection";
	m_debugTcpConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
	connect(m_debugTcpConnection, SIGNAL(disconnected()), this, SLOT(onDebugTcpDisconnection()));
	connect(m_debugTcpConnection, SIGNAL(readyRead()), this, SLOT(onDebugTcpNetworkRead()));
}

void CopterCtrl::onDebugTcpDisconnection()
{
	qDebug() << "Existing debug connection disconnected";
	m_debugTcpConnection->deleteLater();
	m_debugTcpConnection = NULL;
}

void CopterCtrl::onDebugTcpNetworkRead()
{
	if (m_debugTcpConnection.isNull())
		return;
	
}

void CopterCtrl::onButtonRead()
{
	struct input_event evt;
	
	if (read(m_buttonsInputFd, reinterpret_cast<char*>(&evt), sizeof(evt)) != sizeof(evt))
	{
		qDebug() << "Incomplete buttons data read";
		return;
	}
	
	if (evt.type != EV_KEY)
	{
		if (evt.type != EV_SYN)
			qDebug() << "Input event type is not EV_KEY or EV_SYN: " << evt.type;
		return;
	}
	
	BoardButton button;
	
	switch (evt.code) {
		case KEY_F1: button = Button1; break;
		case KEY_F2: button = Button2; break;
		case KEY_F3: button = Button3; break;
		case KEY_F4: button = Button4; break;
		case KEY_F5: button = Button5; break;
		case KEY_F6: button = Button6; break;
		case KEY_F7: button = Button7; break;
		case KEY_F8: button = Button8; break;
	}
	
	tcpLog("Button pressed: " + QString::number(button));
	
	if (static_cast<bool>(evt.value)) {
		emit buttonPressed(button);
	}
	else {
		emit buttonReleased(button);
	}
}

