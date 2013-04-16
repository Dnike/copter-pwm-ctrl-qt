#ifndef COPTERCTRL_HPP
#define COPTERCTRL_HPP

#include <QTcpServer>
#include <QPointer>
#include <QSocketNotifier>
#include <QSettings>
#include <QVector3D>

#include "CopterAxis.hpp"
#if QT_VERSION >= 0x050000
#include <QApplication>
#else
#include <QtGui/QApplication>
#endif

QT_FORWARD_DECLARE_CLASS(Accelerometer)

class CopterCtrl : public QObject
{
	Q_OBJECT
public:
	CopterCtrl();

	float tiltX() const { return m_axisX->tilt(); }
	float tiltY() const { return m_axisY->tilt(); }
	void tiltX(float _tilt) const { m_axisX->tilt(_tilt); m_axisX->setPower(m_power); }
	void tiltY(float _tilt) const { m_axisY->tilt(_tilt); m_axisY->setPower(m_power); }
	void adjustTilt(float tiltX, float tiltY) const { QVector3D tilt(tiltX, tiltY, 0); adjustTilt(tilt); }
	void adjustTilt(QVector3D tilt) const;
	void adjustPower(int _incr);
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
	enum Motor {
		MotorX1,
		MotorX2,
		MotorY1,
		MotorY2,
		MotorAll
	};

public slots:
	void setState(CopterState _state = IDLE) { m_state = _state; emit stateChanged(m_state); }
	void setupAccelZeroAxis();
	void handleTilt(QVector3D tilt);
	void tcpLog(const QString& message);
	void debugTcpLog(const QString& message);
	void emergencyStop();

protected slots:
	void onAccelerometerRead(QVector3D val);
	void onTcpConnection();
	void onTcpDisconnection();
	void onTcpNetworkRead();
	void onDebugTcpConnection();
	void onDebugTcpDisconnection();
	void onDebugTcpNetworkRead();
	void onButtonRead();
	void initMotors(const QString& motorControlPath);
	void initSettings();
	void onMotorPowerChange(float power);
	void adjustSettingsValue(const QString& key, QMetaType::Type type, bool increase = true);
	void onSettingsValueChange(const QString& key, const QVariant& value);

signals:
	void stateChanged(CopterState state);
	void accelerometerRead(QVector3D val);
	void buttonPressed(BoardButton button);
	void buttonReleased(BoardButton button);
	void motorPowerChanged(CopterCtrl::Motor motor, float power);
	void settingsValueChanged(QString key, QVariant value);

protected:
	int m_power;
	CopterAxis* m_axisX;
	CopterAxis* m_axisY;
	QSettings* m_settings;

	QVector3D m_lastDerivative;

	QMap<CopterMotor*, Motor> m_motorIds;

	// TCP network (only in character mode for now)
	QTcpServer           m_tcpServer;
	QPointer<QTcpSocket> m_tcpConnection;
	// Debug tcp network
	QTcpServer           m_debugTcpServer;
	QPointer<QTcpSocket> m_debugTcpConnection;
	// On-board buttons
	int                  m_buttonsInputFd;
	QPointer<QSocketNotifier> m_buttonsInputNotifier;

	QVector3D m_lastTilt;
	QVector<QVector3D> m_pidIntegralVector;
	QVector3D m_pidIntegral;
	unsigned int m_pidCounter;

	Accelerometer* m_accel;
	CopterState m_state;
};

#endif // COPTERCTRL_HPP
