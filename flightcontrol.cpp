#include "flightcontrol.hpp"
#include <cmath>
#include <QTimer>

FlightControl::FlightControl(CopterCtrl* copterCtrl) :
  power(0),
  pidCounter(0),
  pidIntegral(),
  lastDerivative(),
  copterCtrl(copterCtrl),
  lastTime(QTime::currentTime()),
  lastLambda(),
  QObject(copterCtrl)
{
	initMotors();
	// PID setup
	pidIntegralVector = QVector<QVector3D>(copterCtrl->getSettings()->value("PidIWindow").toInt(), QVector3D());
	// accelerometer setup
	accel = QSharedPointer<Accelerometer>(
	            new Accelerometer(copterCtrl->getSettings()->value("AccelInputPath").toString(), copterCtrl, this));
	connect(accel.data(), SIGNAL(dataRead(QVector3D)), this, SIGNAL(accelerometerRead(QVector3D)));
	connect(accel.data(), SIGNAL(dataRead(QVector3D)), this, SLOT(onAccelerometerRead(QVector3D)));

	QTimer* t = new QTimer(this);
	connect(t, SIGNAL(timeout()), this, SLOT(handleTilt()));
	t->start(10);
	// gyro setup
	gyro = QSharedPointer<Gyro>(
	           new Gyro(copterCtrl->getSettings()->value("GyroInputPath").toString(), copterCtrl, this));
	connect(gyro.data(), SIGNAL(dataRead(QVector3D)), this, SLOT(onGyroRead(QVector3D)));
}

void FlightControl::initMotors()
{
	// initialize axes and motors; motors' paths defined in config
	int powerMax = copterCtrl->getSettings()->value("MotorMax").toInt();
	int powerMin = copterCtrl->getSettings()->value("MotorMin").toInt();
	int powerZero = copterCtrl->getSettings()->value("MotorZero").toInt();
	QString motorPathX1 = copterCtrl->getSettings()->value("MotorPathX1").toString();
	QString motorPathX2 = copterCtrl->getSettings()->value("MotorPathX2").toString();
	QString motorPathY1 = copterCtrl->getSettings()->value("MotorPathY1").toString();
	QString motorPathY2 = copterCtrl->getSettings()->value("MotorPathY2").toString();
	CopterMotor* mx1 = new CopterMotor(powerZero, powerMin, powerMax, motorPathX1);
	CopterMotor* mx2 = new CopterMotor(powerZero, powerMin, powerMax, motorPathX2);
	CopterMotor* my1 = new CopterMotor(powerZero, powerMin, powerMax, motorPathY1);
	CopterMotor* my2 = new CopterMotor(powerZero, powerMin, powerMax, motorPathY2);
	motorIds.insert(mx1, MotorX1);
	motorIds.insert(mx2, MotorX2);
	motorIds.insert(my1, MotorY1);
	motorIds.insert(my2, MotorY2);
	connect(mx1, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(mx2, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(my1, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(my2, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));

	motorBase = new CopterMotorBase(mx1, mx2, my1, my2, copterCtrl->getSettings());
}

void FlightControl::onMotorPowerChange(float power)
{
	emit motorPowerChanged(motorIds[dynamic_cast<CopterMotor*>(sender())], power);
}

void FlightControl::adjustTilt(QVector3D tilt) const
{
	// apply power adjusment
	motorBase->setTilt(tilt);
}

void FlightControl::adjustPower(int _incr)
{

	power += _incr;
	power = qMin(100,qMax(power,0));
	emit motorPowerChanged(MotorAll, power);
	copterCtrl->tcpLog("Motor power changed: " + QString::number(power));

	motorBase->setPower(power);
}

void FlightControl::setupAccelZeroAxis()
{
	// deprecated, rewrite to fit new model
	copterCtrl->tcpLog("Start zero axis setup");
	accel->adjustZeroAxis();
}

void FlightControl::onAccelerometerRead(QVector3D val)
{
}

void FlightControl::onGyroRead(QVector3D val)
{

}

QQuaternion FlightControl::dLambda(QQuaternion lambda0, QQuaternion omega)
{
	return (lambda0 * omega + lambda0 * (1 - lambda0.lengthSquared())) / 2;
}

QQuaternion FlightControl::getLambda(QQuaternion lambda0, QQuaternion omega, qreal dt)
{
	// solving diff equation with Runge-Kutt method
	QQuaternion k1, k2, k3, k4;
	k1 = dLambda(lambda0, omega) * dt;
	k2 = dLambda(lambda0 + k1 * 0.5, omega) * dt;
	k3 = dLambda(lambda0 + k2 * 0.5, omega) * dt;
	k4 = dLambda(lambda0 + k3, omega) * dt;
	return lambda0 + (k1 + k2 * 2 + k3 * 2 + k4) / 6;
}

QQuaternion FlightControl::EulerToQuaternion(QVector3D accel_data, QQuaternion lambda) {
	QQuaternion accel(0,accel_data);
	QQuaternion g(0,0,0,1);
	QQuaternion n_axis = (accel*g - g*accel) / 2;
	QQuaternion t_axis = (accel + g) / 2;

	QQuaternion a = lambda;
	a.setScalar(0);
	if (a.length()<0.00001) return QQuaternion(1,0,0,0);

	QQuaternion h = (t_axis*n_axis - n_axis*t_axis) / 2;
	if (h.length()<0.00001) return lambda;
    h.normalize();

	qreal k = (a*h + h*a).scalar() / 2;
	a += h * k;
	a.normalize();

	n_axis = (accel*a - a*accel) / 2;
	n_axis.normalize();
	t_axis = (g*a - a*g) / 2;
	t_axis.normalize();

	h = n_axis + t_axis;
	h.normalize();

	QQuaternion p = h*t_axis - t_axis*h;

	qreal sin_phi = p.length() / 2;
	a *= sin_phi;
	a.setScalar( -(h*t_axis + t_axis*h).scalar() / 2 );
	if (p.x()*a.x() < 0 || p.y()*a.y() < 0) a.setScalar(-a.scalar());

	return a;
}

QVector3D FlightControl::getAngles(QQuaternion lambda) {
	qreal cos_xz = 2*lambda.x()*lambda.z() - 2*lambda.scalar()*lambda.y();
	qreal cos_xy = 2*lambda.x()*lambda.y() + 2*lambda.scalar()*lambda.z();

	qreal roll = acos( cos_xz ) - M_PI/2;
	qreal pitch = acos( 2*lambda.y()*lambda.z() + 2*lambda.scalar()*lambda.x()) - M_PI/2;

	cos_xz *= cos_xz;
	qreal yaw;
	if (cos_xz < 1-1e-6) yaw = acos( (2*lambda.x()*lambda.x() + 2*lambda.scalar()*lambda.scalar() - 1) / sqrt(1-cos_xz) );
	else yaw = 0;
	if (cos_xy > 0) yaw *= -1;

	return QVector3D(roll, pitch, yaw);
}

void FlightControl::handleTilt()
{
	// periodic method, calculates power adjustments from accel ang gyro values
	QTime curTime = QTime::currentTime();
	qreal dt = lastTime.msecsTo(curTime) / 1000.0;
	lastTime = curTime;

	QQuaternion omega(0, gyro->getLastVal() / copterCtrl->getSettings()->value("GyroMappingCoeff").toReal());
	QQuaternion lambda = getLambda(lastLambda, omega, dt);

    QVector3D accel_data = accel->getLastVal().normalized();

    accel_data.setZ( -accel_data.z() );
    qreal temp = accel_data.x();
    accel_data.setX( -accel_data.y() );
    accel_data.setY( -temp );

	QQuaternion lambda_accel = EulerToQuaternion(accel_data, lambda);
	qreal accelCoeff = copterCtrl->getSettings()->value("AccelCorrectingCoeff").toReal();
	lambda = lambda * (1 - accelCoeff) + lambda_accel * accelCoeff;

	lastLambda = lambda;

	// compute PID adjustment
	qreal pidP = copterCtrl->getSettings()->value("PidP").toReal();
	qreal pidI = copterCtrl->getSettings()->value("PidI").toReal();
	qreal pidD = copterCtrl->getSettings()->value("PidD").toReal();

	QVector3D tilt = getAngles(lambda);

	pidIntegral = pidIntegral + tilt - pidIntegralVector[pidCounter];
	pidIntegralVector[pidCounter] = tilt;
	pidCounter = (pidCounter + 1) % (copterCtrl->getSettings()->value("PidIWindow").toInt());

	QVector3D derivative = tilt - lastTilt;
	qreal k = copterCtrl->getSettings()->value("DerivativeK").toReal();
	derivative = lastDerivative + k * (derivative - lastDerivative);
	lastDerivative = derivative;
	QVector3D adj = tilt * pidP + pidIntegral * pidI + derivative * pidD;

	adjustTilt(adj);
	lastTilt = tilt;

	// debug log (telemetrics)
	QStringList debugLineList;
	QVector3D gyro_data = gyro->getLastVal();
	debugLineList << QString::number(lambda.scalar()) << QString::number(lambda.x()) <<
                     QString::number(lambda.y()) << QString::number(lambda.z()) <<
                     QString::number(lambda_accel.scalar()) << QString::number(lambda_accel.x()) <<
                     QString::number(lambda_accel.y()) << QString::number(lambda_accel.z()) <<
	                 //QString::number(motorBase->motorPowerAll())  <<
	                 QString::number(dt*1000)  <<
	                 QString::number(motorBase->motorPowerX1()) << QString::number(motorBase->motorPowerX2())  <<
	                 QString::number(motorBase->motorPowerY1()) << QString::number(motorBase->motorPowerY2());
    /*debugLineList << QString::number(lambda.scalar()) << QString::number(omega.x()) <<
                     QString::number(omega.y()) << QString::number(omega.z()) <<
                     QString::number(lambda_accel.scalar()) << QString::number(accel_data.x()) <<
                     QString::number(accel_data.y()) << QString::number(accel_data.z()) <<
	                 QString::number(power)  <<
	                 QString::number(axisX->motorPower1()) << QString::number(axisX->motorPower2())  <<
	                 QString::number(axisY->motorPower1()) << QString::number(axisY->motorPower2());*/
	copterCtrl->debugTcpLog(debugLineList.join(","));
}

void FlightControl::emergencyStop()
{
	motorBase->emergencyStop();
	QApplication::quit();
}
