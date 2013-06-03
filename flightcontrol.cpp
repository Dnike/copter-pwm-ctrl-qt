#include "flightcontrol.hpp"
#include <cmath>
#include <QTimer>

FlightControl::FlightControl(CopterCtrl* copterCtrl) :
  m_power(0),
  m_pidCounter(0),
  m_pidIntegral(),
  m_lastDerivative(),
  m_copterCtrl(copterCtrl),
  m_lastTime(QTime::currentTime()),
  m_lastLambda(),
  QObject(copterCtrl)
{
	initMotors();
	// PID setup
	m_pidIntegralVector = QVector<QVector3D>(m_copterCtrl->getSettings()->value("PidIWindow").toInt(), QVector3D());
	// accelerometer setup
	m_accel = QSharedPointer<Accelerometer>(
	            new Accelerometer(m_copterCtrl->getSettings()->value("AccelInputPath").toString(), m_copterCtrl, this));
	connect(m_accel.data(), SIGNAL(dataRead(QVector3D)), this, SIGNAL(accelerometerRead(QVector3D)));
	connect(m_accel.data(), SIGNAL(dataRead(QVector3D)), this, SLOT(onAccelerometerRead(QVector3D)));

	QTimer* t = new QTimer(this);
	connect(t, SIGNAL(timeout()), this, SLOT(handleTilt()));
	t->start(10);
	// gyro setup
	m_gyro = QSharedPointer<Gyro>(
	           new Gyro(m_copterCtrl->getSettings()->value("GyroInputPath").toString(), m_copterCtrl, this));
	connect(m_gyro.data(), SIGNAL(dataRead(QVector3D)), this, SLOT(onGyroRead(QVector3D)));
}

void FlightControl::initMotors()
{
	// initialize axes and motors; motors' paths defined in config
	int powerMax = m_copterCtrl->getSettings()->value("MotorMax").toInt();
	int powerMin = m_copterCtrl->getSettings()->value("MotorMin").toInt();
	QString motorPathX1 = m_copterCtrl->getSettings()->value("MotorPathX1").toString();
	QString motorPathX2 = m_copterCtrl->getSettings()->value("MotorPathX2").toString();
	QString motorPathY1 = m_copterCtrl->getSettings()->value("MotorPathY1").toString();
	QString motorPathY2 = m_copterCtrl->getSettings()->value("MotorPathY2").toString();
	CopterMotor* mx1 = new CopterMotor(powerMin, powerMax, motorPathX1);
	CopterMotor* mx2 = new CopterMotor(powerMin, powerMax, motorPathX2);
	CopterMotor* my1 = new CopterMotor(powerMin, powerMax, motorPathY1);
	CopterMotor* my2 = new CopterMotor(powerMin, powerMax, motorPathY2);
	m_motorIds.insert(mx1, MotorX1);
	m_motorIds.insert(mx2, MotorX2);
	m_motorIds.insert(my1, MotorY1);
	m_motorIds.insert(my2, MotorY2);
	connect(mx1, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(mx2, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(my1, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));
	connect(my2, SIGNAL(powerChanged(float)), this, SLOT(onMotorPowerChange(float)));

	m_axisX = new CopterAxis(mx1, mx2, m_copterCtrl->getSettings());
	m_axisY = new CopterAxis(my1, my2, m_copterCtrl->getSettings());
}

void FlightControl::onMotorPowerChange(float power)
{
	emit motorPowerChanged(m_motorIds[dynamic_cast<CopterMotor*>(sender())], power);
}

void FlightControl::adjustTilt(QVector3D tilt) const
{
	// apply power adjusment
	m_axisX->tilt(tilt.x());
	m_axisY->tilt(tilt.y());
}

void FlightControl::adjustPower(int _incr)
{
	static const int s_power_min = m_copterCtrl->getSettings()->value("PowerMin").toInt();
	static const int s_power_max = m_copterCtrl->getSettings()->value("PowerMax").toInt();

	m_power += _incr;
	m_power = qMax(qMin(m_power, s_power_max), s_power_min);
	emit motorPowerChanged(MotorAll, m_power);
	m_copterCtrl->tcpLog("Motor power changed: " + QString::number(m_power));

	m_axisX->setPower(m_power);
	m_axisY->setPower(m_power);
}

void FlightControl::setupAccelZeroAxis()
{
	// deprecated, rewrite to fit new model
	m_copterCtrl->tcpLog("Start zero axis setup");
	m_accel->adjustZeroAxis();
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
	if (a.length()<0.001) return QQuaternion(1,0,0,0);

	QQuaternion h = (t_axis*n_axis - n_axis*t_axis) / 2;
	if (h.length()<0.001) return lambda;
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
	a.setScalar( -(h*n_axis + n_axis*h).scalar() / 2 );
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
	qreal dt = m_lastTime.msecsTo(curTime) / 1000.0;
	m_lastTime = curTime;

	QQuaternion omega(0, m_gyro->getLastVal() / m_copterCtrl->getSettings()->value("GyroMappingCoeff").toReal());
	QQuaternion lambda = getLambda(m_lastLambda, omega, dt);

    QVector3D accel_data = m_accel->getLastVal().normalized();
    accel_data.setX( -accel_data.x() );
    accel_data.setY( -accel_data.y() );

	QQuaternion lambda_accel = EulerToQuaternion(accel_data, lambda);
	qreal accelCoeff = m_copterCtrl->getSettings()->value("AccelCorrectingCoeff").toReal();
	lambda = lambda * (1 - accelCoeff) + lambda_accel * accelCoeff;

	m_lastLambda = lambda;

	// compute PID adjustment
	qreal pidP = m_copterCtrl->getSettings()->value("PidP").toReal();
	qreal pidI = m_copterCtrl->getSettings()->value("PidI").toReal();
	qreal pidD = m_copterCtrl->getSettings()->value("PidD").toReal();

	QVector3D tilt = getAngles(lambda);

	m_pidIntegral = m_pidIntegral + tilt - m_pidIntegralVector[m_pidCounter];
	m_pidIntegralVector[m_pidCounter] = tilt;
	m_pidCounter = (m_pidCounter + 1) % (m_copterCtrl->getSettings()->value("PidIWindow").toInt());

	QVector3D derivative = tilt - m_lastTilt;
	qreal k = m_copterCtrl->getSettings()->value("DerivativeK").toReal();
	derivative = m_lastDerivative + k * (derivative - m_lastDerivative);
	m_lastDerivative = derivative;
	QVector3D adj = tilt * pidP + m_pidIntegral * pidI + derivative * pidD;

	adjustTilt(adj);
	m_lastTilt = tilt;

	// debug log (telemetrics)
	QStringList debugLineList;
	QVector3D gyro = m_gyro->getLastVal();
	debugLineList << QString::number(lambda.scalar()) << QString::number(lambda.x()) <<
                     QString::number(lambda.y()) << QString::number(lambda.z()) <<
                     QString::number(lambda_accel.scalar()) << QString::number(lambda_accel.x()) <<
                     QString::number(lambda_accel.y()) << QString::number(lambda_accel.z()) <<
	                 QString::number(m_power)  <<
	                 QString::number(m_axisX->motorPower1()) << QString::number(m_axisX->motorPower2())  <<
	                 QString::number(m_axisY->motorPower1()) << QString::number(m_axisY->motorPower2());
	m_copterCtrl->debugTcpLog(debugLineList.join(","));
}

void FlightControl::emergencyStop()
{
	m_axisX->emergencyStop();
	m_axisY->emergencyStop();
	QApplication::quit();
}
