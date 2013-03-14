#pragma once
#include "CopterMotor.hpp"

class CopterAxis : public QObject
{
	Q_OBJECT
public:
	CopterAxis(CopterMotor* _motor1, CopterMotor* _motor2, QSettings* settings);

	float tilt() const;
	void tilt(float _tilt);

	void setPower(int _power);

public slots:
	void emergencyStop();
	void updateMotorsPower();

private:
	int m_power;
	float m_tilt;
	CopterMotor* m_motor1;
	CopterMotor* m_motor2;
	QSettings* m_settings;
};
