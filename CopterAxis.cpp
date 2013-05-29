#include "CopterAxis.hpp"

CopterAxis::CopterAxis(CopterMotor *_motor1, CopterMotor *_motor2, QSharedPointer<QSettings> settings) :
	m_motor1(_motor1),
	m_motor2(_motor2),
	m_tilt(0),
	m_power(0),
	m_settings(settings)
{
}

float CopterAxis::tilt() const
{
	return m_tilt;
}

void CopterAxis::tilt(float _tilt)
{
	m_tilt = _tilt;
	updateMotorsPower();
}

void CopterAxis::setPower(int _power)
{
	m_power = _power;
	updateMotorsPower();
}

void CopterAxis::emergencyStop()
{
	m_motor1->emergencyStop();
	m_motor2->emergencyStop();
}

void CopterAxis::updateMotorsPower()
{
	// applying power adjustments
	int power1 = floor(sqrt(static_cast<float>(m_power * m_power) + m_tilt));
	int power2 = floor(sqrt(static_cast<float>(m_power * m_power) - m_tilt));
	float alpha = m_settings->value("MotorIntervalAlpha").toFloat();
	// cap so that motors won't stop
	int upperBound = floor(m_power * (1 + alpha));
	int lowerBound = floor(m_power * (1 - alpha));
	power1 = qMax(lowerBound, qMin(power1, upperBound));
	power2 = qMax(lowerBound, qMin(power2, upperBound));
	// cap to fit the interval (PowerMin, PowerMax) (usually (0, 100))
	int powerMax = m_settings->value("PowerMax").toInt();
	int powerMin = m_settings->value("PowerMin").toInt();
	power1 = qMax(powerMin, qMin(powerMax, power1));
	power2 = qMax(powerMin, qMin(powerMax, power2));

	// applying power
	m_motor1->setPower(power1);
	m_motor2->setPower(power2);
}
