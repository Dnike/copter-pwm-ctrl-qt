#ifndef FLIGHTCONTROL_HPP
#define FLIGHTCONTROL_HPP

#include "CopterCtrl.hpp"
#include "accelerometer.hpp"
#include "gyro.hpp"
#include "CopterMotorBase.hpp"

#include <QQuaternion>
#include <QTime>

class FlightControl : public QObject
{
	Q_OBJECT
public:
	explicit FlightControl(CopterCtrl* CopterCtrl);

	void adjustTilt(QVector3D tilt) const;
	void adjustPower(int _incr);

	enum Motor {
		MotorX1,
		MotorX2,
		MotorY1,
		MotorY2,
		MotorAll
	};

	QQuaternion dLambda(QQuaternion lambda0, QQuaternion omega);
	QQuaternion getLambda(QQuaternion lambda0, QQuaternion omega, qreal dt);
	QQuaternion EulerToQuaternion(QVector3D accel_data, QQuaternion lambda);
	QVector3D getAngles(QQuaternion lambda);
signals:
	void accelerometerRead(QVector3D val);
	void motorPowerChanged(FlightControl::Motor motor, float power);

public slots:
	void emergencyStop();
	void setupAccelZeroAxis();

protected slots:
	void onAccelerometerRead(QVector3D val);
	void onGyroRead(QVector3D val);
	void initMotors();
	void onMotorPowerChange(float power);
	void handleTilt();

private:
	int power;
	CopterMotorBase* motorBase;

	QVector3D lastDerivative;
	QMap<CopterMotor*, Motor> motorIds;

	QVector3D lastTilt;
	QVector<QVector3D> pidIntegralVector;
	QVector3D pidIntegral;
	unsigned int pidCounter;

	QSharedPointer<Accelerometer> accel;
	QSharedPointer<Gyro> gyro;
	CopterCtrl* copterCtrl;

	QTime lastTime;
	QQuaternion lastLambda;
};

#endif // FLIGHTCONTROL_HPP
