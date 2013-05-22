#ifndef COPTERCTRL_HPP
#define COPTERCTRL_HPP

#include <QTcpServer>
#include <QPointer>
#include <QSocketNotifier>
#include <QSettings>
#include <QVector3D>
#include <QSharedPointer>

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
	~CopterCtrl();
	
	
	QSharedPointer<QSettings> getSettings() { return m_settings; }
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
	void buttonPressed(BoardButton button);
	void buttonReleased(BoardButton button);
	void settingsValueChanged(const QString& key, const QVariant& value);

public slots:
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
	QSharedPointer<QSettings> m_settings;

	// TCP network (only in character mode for now)
	QTcpServer           m_tcpServer;
	QPointer<QTcpSocket> m_tcpConnection;
	// Debug tcp network
	QTcpServer           m_debugTcpServer;
	QPointer<QTcpSocket> m_debugTcpConnection;
	// On-board buttons
	int                  m_buttonsInputFd;
	QPointer<QSocketNotifier> m_buttonsInputNotifier;

	QSharedPointer<FlightControl> m_flightControl;
};

#endif // COPTERCTRL_HPP
