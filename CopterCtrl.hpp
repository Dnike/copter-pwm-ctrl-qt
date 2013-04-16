#ifndef COPTERCTRL_HPP
#define COPTERCTRL_HPP

#include <QTcpServer>
#include <QPointer>
#include <QSocketNotifier>
#include <QSettings>
#include <QVector3D>

#if QT_VERSION >= 0x050000
#include <QApplication>
#else
#include <QtGui/QApplication>
#endif

QT_FORWARD_DECLARE_CLASS(Accelerometer)
QT_FORWARD_DECLARE_CLASS(Gyro)
QT_FORWARD_DECLARE_CLASS(FlightControl)

class CopterCtrl : public QObject
{
	Q_OBJECT
public:
	CopterCtrl();

	enum CopterState { IDLE = 0,
										 NUM_STATES };
	const CopterState state() { return m_state; }
	const QString stateString() {
		switch (m_state) {
			case IDLE: return QString("Idling..."); break;
			default: return QString("Unknown status"); break;
		}
	}
	QSettings* getSettings() { return m_settings; }
	enum BoardButton {
		Button1 = 0,
		Button2,
		Button3,
		Button4,
		Button5,
		Button6,
		Button7,
		Button8,
		NUM_BUTTONS
	};

signals:
	void stateChanged(CopterState state);
	void buttonPressed(BoardButton button);
	void buttonReleased(BoardButton button);
	void settingsValueChanged(QString key, QVariant value);

public slots:
	void setState(CopterState _state = IDLE) { m_state = _state; emit stateChanged(m_state); }
	void tcpLog(const QString& message);
	void debugTcpLog(const QString& message);
	void emergencyStop();

protected slots:
	void onTcpConnection();
	void onTcpDisconnection();
	void onTcpNetworkRead();
	void onDebugTcpConnection();
	void onDebugTcpDisconnection();
	void onDebugTcpNetworkRead();
	void onButtonRead();
	void initSettings();
	void adjustSettingsValue(const QString& key, QMetaType::Type type, bool increase = true);
	void onSettingsValueChange(const QString& key, const QVariant& value);

protected:
	QSettings* m_settings;

	// TCP network (only in character mode for now)
	QTcpServer           m_tcpServer;
	QPointer<QTcpSocket> m_tcpConnection;
	// Debug tcp network
	QTcpServer           m_debugTcpServer;
	QPointer<QTcpSocket> m_debugTcpConnection;
	// On-board buttons
	int                  m_buttonsInputFd;
	QPointer<QSocketNotifier> m_buttonsInputNotifier;

	FlightControl* m_flightControl;
	CopterState m_state;
};

#endif // COPTERCTRL_HPP
