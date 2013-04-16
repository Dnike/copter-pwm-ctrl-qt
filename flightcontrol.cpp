#include "flightcontrol.hpp"

FlightControl::FlightControl(CopterCtrl* copterCtrl) :
	m_power(0),
	m_pidCounter(0),
	m_pidIntegral(),
	m_lastDerivative(),
	m_copterCtrl(copterCtrl),
	QObject(copterCtrl)
{
	initMotors(m_copterCtrl->getSettings()->value("ControlPath").toString());
	// PID setup
	m_pidIntegralVector = QVector<QVector3D>(m_copterCtrl->getSettings()->value("PidIWindow").toInt(), QVector3D());
	// accelerometer setup
	m_accel = new Accelerometer(m_copterCtrl->getSettings()->value("AccelInputPath").toString(), m_copterCtrl, this);
	connect(m_accel, SIGNAL(dataRead(QVector3D)), this, SIGNAL(accelerometerRead(QVector3D)));
	connect(m_accel, SIGNAL(dataRead(QVector3D)), this, SLOT(onAccelerometerRead(QVector3D)));
	connect(m_accel, SIGNAL(dataRead(QVector3D)), this, SLOT(handleTilt(QVector3D)));
	// gyro setup
	m_gyro = new Gyro(m_copterCtrl->getSettings()->value("GyroInputPath").toString(), m_copterCtrl, this);
	connect(m_gyro, SIGNAL(dataRead(QVector3D)), this, SLOT(onGyroRead(QVector3D)));
}

void FlightControl::initMotors(const QString& motorControlPath)
{
	QString motorControlFile = m_copterCtrl->getSettings()->value("MotorControlFile").toString();
	int powerMax = m_copterCtrl->getSettings()->value("MotorMax").toInt();
	int powerMin = m_copterCtrl->getSettings()->value("MotorMin").toInt();
	CopterMotor* mx1 = new CopterMotor(powerMin, powerMax, motorControlPath + "ehrpwm.0/pwm/ehrpwm.0:0/" + motorControlFile);
	CopterMotor* mx2 = new CopterMotor(powerMin, powerMax, motorControlPath + "ehrpwm.0/pwm/ehrpwm.0:1/" + motorControlFile);
	CopterMotor* my1 = new CopterMotor(powerMin, powerMax, motorControlPath + "ehrpwm.1/pwm/ehrpwm.1:0/" + motorControlFile);
	CopterMotor* my2 = new CopterMotor(powerMin, powerMax, motorControlPath + "ehrpwm.1/pwm/ehrpwm.1:1/" + motorControlFile);
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
	m_copterCtrl->tcpLog("Start zero axis setup");
	m_accel->adjustZeroAxis();
}

void FlightControl::onAccelerometerRead(QVector3D val)
{
}

void FlightControl::onGyroRead(QVector3D val)
{

}

void FlightControl::handleTilt(QVector3D tilt)
{
	float pidP = m_copterCtrl->getSettings()->value("PidP").toFloat();
	float pidI = m_copterCtrl->getSettings()->value("PidI").toFloat();
	float pidD = m_copterCtrl->getSettings()->value("PidD").toFloat();

	// TODO: handle case when window or coeff = 0
	m_pidIntegral = m_pidIntegral + tilt - m_pidIntegralVector[m_pidCounter];
	m_pidIntegralVector[m_pidCounter] = tilt;
	m_pidCounter = (m_pidCounter + 1) % (m_copterCtrl->getSettings()->value("PidIWindow").toInt());
	//	float x = ((tilt.x() > 0) ? 1 : -1) * sqrt(fabs(tilt.x()));
	//	float y = ((tilt.y() > 0) ? 1 : -1) * sqrt(fabs(tilt.y()));
	//	float z = ((tilt.z() > 0) ? 1 : -1) * sqrt(fabs(tilt.z()));
	//	QVector3D psqrt = QVector3D(x, y, z);
	//	QVector3D adj = psqrt * pidP + m_pidIntegral * pidI + (tilt - m_lastTilt) * pidD;
	QVector3D derivative = tilt - m_lastTilt;
	float k = m_copterCtrl->getSettings()->value("DerivativeK").toFloat();
	derivative = m_lastDerivative + k * (derivative - m_lastDerivative);
	m_lastDerivative = derivative;
	QVector3D adj = tilt * pidP + m_pidIntegral * pidI + derivative * pidD;

	adjustTilt(adj);
	m_lastTilt = tilt;

	// debug log
	QStringList debugLineList;
	// temporary
	QVector3D gyro = m_gyro->getLastVal();
	debugLineList << QString::number(tilt.x()) << QString::number(tilt.y()) << QString::number(tilt.z()) <<
									 QString::number(gyro.x()) << QString::number(gyro.y()) << QString::number(gyro.z()) <<
									 QString::number(adj.x())  << QString::number(adj.y())  << QString::number(adj.z())  <<
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
