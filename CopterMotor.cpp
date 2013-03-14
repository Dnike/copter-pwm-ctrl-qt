#include "CopterMotor.hpp"

void CopterMotor::invoke_open()
{
	invoke(0);
}

void CopterMotor::invoke_close()
{
	invoke(0);
}

void CopterMotor::invoke(int _power)
{
	QString s;
	m_ctrlFile.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Unbuffered|QIODevice::Text);
	float powerFactor = (float)(m_powerMax - m_powerMin) / 100;
	s.sprintf("%d\n", static_cast<int>(_power * powerFactor + m_powerMin));
	m_ctrlFile.write(s.toLatin1());
	m_ctrlFile.close();
}

CopterMotor::CopterMotor(QSettings* settings, const QString& _ctrlPath) :
	m_settings(settings),
	m_ctrlFile(_ctrlPath),
	m_delta(1.0)
{
	m_powerMin = m_settings->value("MotorMin").toInt();
	m_powerMax = m_settings->value("MotorMax").toInt();

	invoke_open();
}

CopterMotor::~CopterMotor()
{
	invoke_close();
}

void CopterMotor::setPower(unsigned int _power)
{
	int powerMax = m_settings->value("PowerMax").toInt();
	int powerMin = m_settings->value("PowerMin").toInt();
	_power = qMax(powerMin, qMin(powerMax, static_cast<int>(_power)));
	invoke(_power);
	emit powerChanged(static_cast<float>(_power));
}


void CopterMotor::emergencyStop()
{
	invoke(0);
}

