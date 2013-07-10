#pragma once
#include "CopterMotor.hpp"
#include <QVector3D>

class CopterMotorBase : public QObject
{
	Q_OBJECT
public:
	CopterMotorBase(CopterMotor* _motorX1, CopterMotor* _motorX2,
        CopterMotor* _motorY1, CopterMotor* _motorY2, QSharedPointer<QSettings> settings);

	void setTilt(QVector3D _tilt);

	void setPower(int _power);
	int motorPowerX1() const { return motorX1->getPower()/1000; }
	int motorPowerX2() const { return motorX2->getPower()/1000; }
	int motorPowerY1() const { return motorY1->getPower()/1000; }
	int motorPowerY2() const { return motorY2->getPower()/1000; }
	int motorPowerAll() const { return raw_power/1000; }


public slots:
	void emergencyStop();
	void updateMotorsPower();

private:
    int m_powerMin;
    int m_powerMax;
    int m_powerZero;
	int power, raw_power;
	QVector3D tilt;
	CopterMotor* motorX1;
	CopterMotor* motorX2;
	CopterMotor* motorY1;
	CopterMotor* motorY2;
	QSharedPointer<QSettings> settings;
};
