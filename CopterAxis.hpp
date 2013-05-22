#pragma once
#include "CopterMotor.hpp"

class CopterAxis : public QObject
{
	Q_OBJECT
public:
	CopterAxis(CopterMotor* _motor1, CopterMotor* _motor2, QSharedPointer<QSettings> settings);

	float tilt() const;
	void tilt(float _tilt);

	void setPower(int _power);
	int motorPower1() const { return m_motor1->power(); }
	int motorPower2() const { return m_motor2->power(); }


public slots:
	void emergencyStop();
	void updateMotorsPower();

private:
	int m_power;
	float m_tilt;
	CopterMotor* m_motor1;
	CopterMotor* m_motor2;
	QSharedPointer<QSettings> m_settings;
};
