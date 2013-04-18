#include "CopterMotor.hpp"

#include <QDebug>

void CopterMotor::invoke(int _power)
{
	m_power = _power;
	QString s;
	float powerFactor = (float)(m_powerMax - m_powerMin) / 100;
	s.sprintf("%d\n", static_cast<int>(_power * powerFactor + m_powerMin));
	if (!m_ctrlFile.isOpen()) {
		if (!m_ctrlFile.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Unbuffered|QIODevice::Text)) {
			qDebug() << "Can't open motor control file " + m_ctrlFile.fileName() << endl;
		}
	}
	m_ctrlFile.write(s.toLatin1());
	m_ctrlFile.close();
}

CopterMotor::CopterMotor(int powerMin, int powerMax, const QString& _ctrlPath) :
	m_ctrlFile(_ctrlPath),
	m_delta(1.0),
	m_powerMin(powerMin),
	m_powerMax(powerMax),
	m_power(0)
{
	if (!m_ctrlFile.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Unbuffered|QIODevice::Text)) {
		qDebug() << "Can't open motor control file " + m_ctrlFile.fileName() << endl;
	}
	invoke(0);
}

CopterMotor::~CopterMotor()
{
	invoke(0);
	m_ctrlFile.close();
}

void CopterMotor::setPower(int _power)
{
	invoke(_power);
	emit powerChanged(static_cast<float>(_power));
}


void CopterMotor::emergencyStop()
{
	invoke(0);
}

