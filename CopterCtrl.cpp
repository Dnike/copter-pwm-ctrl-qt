#include <QTimer>
#include <QTcpSocket>

#include <cmath>
#include "CopterCtrl.hpp"
#include "accelerometer.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

CopterCtrl::CopterCtrl() :
	m_power(0),
	m_state(IDLE),
	m_tcpServer(),
	m_tcpConnection(),
	m_pidCounter(0),
	m_pidIntegral()
{
	initSettings();
	initMotors(m_settings->value("ControlPath").toString());

	m_pidIntegralVector = QVector<QVector3D>(m_settings->value("PidIWindow").toInt(), QVector3D());

	m_tcpServer.listen(QHostAddress::Any, m_settings->value("TcpPort").toInt());
	connect(&m_tcpServer, SIGNAL(newConnection()), this, SLOT(onConnection()));

	// buttons reading
	const QString s_buttons_input_path = m_settings->value("ButtonsInputPath").toString();
	m_buttonsInputFd = ::open(s_buttons_input_path.toLatin1().data(), O_SYNC, O_RDONLY);
	if (m_buttonsInputFd == -1)
		qDebug() << "Cannot open buttons input file " << s_buttons_input_path << ", reason: " << errno;

	m_buttonsInputNotifier = new QSocketNotifier(m_buttonsInputFd, QSocketNotifier::Read, this);
	connect(m_buttonsInputNotifier, SIGNAL(activated(int)), this, SLOT(onButtonRead()));
	m_buttonsInputNotifier->setEnabled(true);

	m_accel = new Accelerometer(m_settings->value("AccelInputPath").toString(), this);
	connect(m_accel, SIGNAL(accelerometerRead(QVector3D)),
					this, SIGNAL(accelerometerRead(QVector3D)));
	connect(m_accel, SIGNAL(accelerometerRead(QVector3D)), this, SLOT(handleTilt(QVector3D)));
	connect(m_accel, SIGNAL(zeroAxisChanged(QVector3D)), this, SIGNAL(zeroAxisChanged(QVector3D)));
}

void CopterCtrl::initMotors(const QString& motorControlPath)
{
	QString motorControlFile = m_settings->value("MotorControlFile").toString();
	CopterMotor* mx1 = new CopterMotor(m_settings, motorControlPath + "ehrpwm.0/pwm/ehrpwm.0:0/" + motorControlFile);
	CopterMotor* mx2 = new CopterMotor(m_settings, motorControlPath + "ehrpwm.0/pwm/ehrpwm.0:1/" + motorControlFile);
	CopterMotor* my1 = new CopterMotor(m_settings, motorControlPath + "ehrpwm.1/pwm/ehrpwm.1:0/" + motorControlFile);
	CopterMotor* my2 = new CopterMotor(m_settings, motorControlPath + "ehrpwm.1/pwm/ehrpwm.1:1/" + motorControlFile);
	m_motorIds.insert(mx1, MotorX1);
	m_motorIds.insert(mx2, MotorX2);
	m_motorIds.insert(my1, MotorY1);
	m_motorIds.insert(my2, MotorY2);
	connect(mx1, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(mx2, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(my1, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(my2, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));

	m_axisX = new CopterAxis(mx1, mx2);
	m_axisY = new CopterAxis(my1, my2);
}

void CopterCtrl::initSettings()
{
	m_settings = new QSettings(QApplication::applicationDirPath() + "/config.ini", QSettings::IniFormat);

	// TODO: write proper checker
	if (m_settings->allKeys().count() == 0) {
		// TODO: move to conf file
		m_settings->setValue("ControlPath", "/sys/devices/platform/");
		m_settings->setValue("AccelInputPath", "/dev/input/event1");
		m_settings->setValue("ButtonsInputPath", "/dev/input/event0");
		m_settings->setValue("AccelAdjustingTime", 5000);
		m_settings->setValue("TcpPort", 4000);
		m_settings->setValue("TiltStep", 0.02d);
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
		m_settings->setValue("WriteLog", true);
		m_settings->setValue("MotorControlFile", "duty_ns");
		m_settings->setValue("NoGraphics", false);
	}

	m_settings->setFallbacksEnabled(false);
	m_settings->sync();

	connect(this, SIGNAL(settingsValueChanged(QString,QVariant)), this, SLOT(onSettingsValueChange(QString,QVariant)));
}

void CopterCtrl::onMotorPowerChange(float power)
{
	emit motorPowerChanged(m_motorIds[dynamic_cast<CopterMotor*>(sender())], power);
}

void CopterCtrl::adjustSettingsValue(const QString &key, bool increase)
{
	QVariant::Type type = m_settings->value(key).type();
	switch(type) {
		case QMetaType::Int:
			m_settings->setValue(key, m_settings->value(key).toInt() + (increase ? 1 : -1));
			break;
		case QMetaType::Float:
			m_settings->setValue(key, m_settings->value(key).toFloat() * (increase ? 0.9 : (1.0 / 0.9)));
			break;
		case QMetaType::Double:
			m_settings->setValue(key, m_settings->value(key).toDouble() * (increase ? 0.9 : (1.0 / 0.9)));
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

void CopterCtrl::adjustTilt(QVector3D tilt) const
{
	m_axisX->tilt(m_axisX->tilt() + tilt.x());
	m_axisY->tilt(m_axisY->tilt() + tilt.y());
	m_axisX->setPower(m_power);
	m_axisY->setPower(m_power);
}

void CopterCtrl::adjustPower(int _incr)
{
	static const int s_power_min = m_settings->value("PowerMin").toInt();
	static const int s_power_max = m_settings->value("PowerMax").toInt();

	m_power += _incr;
	m_power = qMax(qMin(m_power, s_power_max), s_power_min);
	emit motorPowerChanged(MotorAll, m_power);
	tcpLog("Motor power changed: " + QString::number(m_power));

	m_axisX->setPower(m_power);
	m_axisY->setPower(m_power);
}

void CopterCtrl::setupAccelZeroAxis()
{
	if (m_state != CopterCtrl::IDLE)
		return;
	tcpLog("Start zero axis setup");
	setState(CopterCtrl::ADJUSTING_ACCEL);

	QTimer::singleShot(m_settings->value("AccelAdjustingTime").toInt(), this, SLOT(setState()));
}

void CopterCtrl::onAccelerometerRead(QVector3D val)
{
}

void CopterCtrl::handleTilt(QVector3D tilt)
{
	float pidP = m_settings->value("PidP").toFloat();
	float pidI = m_settings->value("PidI").toFloat();
	float pidD = m_settings->value("PidD").toFloat();

	// TODO: handle case when window or coeff = 0
	m_pidIntegral = m_pidIntegral + tilt - m_pidIntegralVector[m_pidCounter];
	m_pidIntegralVector[m_pidCounter] = tilt;
	m_pidCounter = (m_pidCounter + 1) % (m_settings->value("PidIWindow").toInt());
	QVector3D adj = tilt * pidP + m_pidIntegral * pidI + (tilt - m_lastTilt) * pidD;

	adjustTilt(adj);
	m_lastTilt = tilt;
}

void CopterCtrl::tcpLog(const QString &message)
{
	if (!m_tcpConnection.isNull()) {
		m_tcpConnection->write(message.toAscii());
		m_tcpConnection->write("\n\r");
	}
}

void CopterCtrl::emergencyStop()
{
	m_axisX->emergencyStop();
	m_axisY->emergencyStop();
	QApplication::quit();
}

void CopterCtrl::onConnection()
{
	if (!m_tcpConnection.isNull())
		qDebug() << "Replacing existing connection";
	m_tcpConnection = m_tcpServer.nextPendingConnection();
	qDebug() << "Accepted new connection";
	m_tcpConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
	connect(m_tcpConnection, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
	connect(m_tcpConnection, SIGNAL(readyRead()), this, SLOT(onNetworkRead()));
}

void CopterCtrl::onDisconnected()
{
	qDebug() << "Existing connection disconnected";
	m_tcpConnection = 0;
}

void CopterCtrl::onNetworkRead()
{
	if (m_tcpConnection.isNull())
		return;

	static const float s_tilt_step = m_settings->value("TiltStep").toFloat();
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
			case '1': adjustTilt(-s_tilt_step, -s_tilt_step); break;
			case '2': adjustTilt(0,            -s_tilt_step); break;
			case '3': adjustTilt(+s_tilt_step, -s_tilt_step); break;
			case '4': adjustTilt(-s_tilt_step, 0); break;
			case '5': tiltX(0); tiltY(0); break;
			case '6': adjustTilt(+s_tilt_step, 0); break;
			case '7': adjustTilt(-s_tilt_step, +s_tilt_step); break;
			case '8': adjustTilt(0,            +s_tilt_step); break;
			case '9': adjustTilt(+s_tilt_step, +s_tilt_step); break;
			case 'Z': adjustPower(-s_power_max); break;
			case 'z': adjustPower(-s_power_step2); break;
			case 'x': adjustPower(-s_power_step1); break;
			case 'c': adjustPower(+s_power_step1); break;
			case 'v': adjustPower(+s_power_step2); break;
			case 'V': adjustPower(+s_power_max); break;
			case '0': setupAccelZeroAxis(); break;
			case 'a': emergencyStop(); break;
			case '[': adjustSettingsValue("MotorMax", false); break;
			case ']': adjustSettingsValue("MotorMax", true); break;
			case '{': adjustSettingsValue("MotorMin", false); break;
			case '}': adjustSettingsValue("MotorMin", true); break;
			case ',': adjustSettingsValue("PidP", false); break;
			case '.': adjustSettingsValue("PidP", true); break;
			case '<': adjustSettingsValue("PidD", false); break;
			case '>': adjustSettingsValue("PidD", true); break;
			case '(': adjustSettingsValue("PidI", false); break;
			case ')': adjustSettingsValue("PidI", true); break;
		}
	}
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

