#include "CopterAxis.hpp"

CopterAxis::CopterAxis(CopterMotor *_motor1, CopterMotor *_motor2) :
	m_motor1(_motor1),
	m_motor2(_motor2),
	m_tilt(0),
	m_power(0)
{
}

float CopterAxis::tilt() const
{
	return m_tilt;
}

void CopterAxis::tilt(float _tilt)
{
	m_tilt = _tilt;
}

void CopterAxis::setPower(unsigned int _power)
{
	m_power = _power;
	unsigned int power1 = floor(static_cast<float>(_power) + m_tilt / 2);
	unsigned int power2 = floor(static_cast<float>(_power) - m_tilt / 2);
	power1 = qMin(power1, m_power * 2);
	power2 = qMin(power2, m_power * 2);
	m_motor1->setPower(power1);
	m_motor2->setPower(power2);
}

void CopterAxis::emergencyStop()
{
	m_motor1->emergencyStop();
	m_motor2->emergencyStop();
}
