#include "CopterMotor.hpp"

#include <QDebug>

void CopterMotor::invoke(int _power)
{
	// applying power to hardware motor
	power = _power;
	QString s;
	// TODO: remove magic number
	s.sprintf("%d\n", _power);
	if (!ctrlFile.isOpen()) {
		if (!ctrlFile.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Unbuffered|QIODevice::Text)) {
			qDebug() << "Can't open motor control file " + ctrlFile.fileName() << endl;
		}
	}
	ctrlFile.write(s.toLatin1());
	ctrlFile.close();
}

CopterMotor::CopterMotor(int _powerZero, int _powerMin, int _powerMax, const QString& _ctrlPath) :
	ctrlFile(_ctrlPath),
	delta(1.0),
	powerZero(_powerZero),
	powerMin(_powerMin),
	powerMax(_powerMax),
	power(_powerZero)
{
	if (!ctrlFile.open(QIODevice::WriteOnly|QIODevice::Truncate|QIODevice::Unbuffered|QIODevice::Text)) {
		qDebug() << "Can't open motor control file " + ctrlFile.fileName() << endl;
	}
	invoke(powerZero);
}

CopterMotor::~CopterMotor()
{
	invoke(powerZero);
	ctrlFile.close();
}

void CopterMotor::setPower(int _power)
{
	invoke(_power);
	emit powerChanged(static_cast<float>(_power));
}


void CopterMotor::emergencyStop()
{
	invoke(powerZero);
}

