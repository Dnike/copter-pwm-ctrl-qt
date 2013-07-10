#include "CopterMotorBase.hpp"

CopterMotorBase::CopterMotorBase(CopterMotor* _motorX1, CopterMotor* _motorX2,
        CopterMotor* _motorY1, CopterMotor* _motorY2, QSharedPointer<QSettings> settings) :

	motorX1(_motorX1),
	motorX2(_motorX2),
	motorY1(_motorY1),
	motorY2(_motorY2),
	tilt(0,0,0),
	settings(settings)
{
	m_powerMax = settings->value("MotorMax").toInt();
	m_powerMin = settings->value("MotorMin").toInt();
	m_powerZero = settings->value("MotorZero").toInt();
	power = 0;
	raw_power = m_powerZero;
}

void CopterMotorBase::setTilt(QVector3D _tilt)
{
	tilt = _tilt;
	updateMotorsPower();
}

void CopterMotorBase::setPower(int _power)
{
	power = _power;
	updateMotorsPower();
}

void CopterMotorBase::emergencyStop()
{
	motorX1->emergencyStop();
	motorX2->emergencyStop();
	motorY1->emergencyStop();
	motorY2->emergencyStop();
}

void CopterMotorBase::updateMotorsPower()
{
	raw_power = power/100.0 * (m_powerMax - m_powerZero) + m_powerZero;
	if (power==0) {
        motorX1->setPower(m_powerZero);
        motorX2->setPower(m_powerZero);
        motorY1->setPower(m_powerZero);
        motorY2->setPower(m_powerZero);
        return;
    }

	// applying power adjustments
	int power_out[4];
	power_out[0] = raw_power + tilt.x();
	power_out[1] = raw_power - tilt.x();
    power_out[2] = raw_power + tilt.y();
	power_out[3] = raw_power - tilt.y();

	int upper_margin = m_powerMax - m_powerMin;
	int lower_margin = upper_margin;
    for (int i = 0; i<4; i++) {
        upper_margin = qMin( upper_margin, m_powerMax - power_out[i]);
        lower_margin = qMin( lower_margin, power_out[i] - m_powerMin);
    }

    if (upper_margin < 0 || lower_margin < 0) {
        int motor_shift = ( upper_margin - lower_margin ) / 2;
        if( upper_margin < 0 ) {
            motor_shift = qMax(upper_margin, motor_shift);
        }

        if( lower_margin < 0 ) {
            motor_shift = qMin(-lower_margin, motor_shift);
        }
        if (motor_shift != 0) {
            for (int i = 0; i<4; i++) power_out[i] += motor_shift;
            raw_power += motor_shift;
        }
    }

    for (int i = 0; i<4; i++) power_out[i] = qMin(m_powerMax, qMax(m_powerMin, power_out[i]));

	// applying power
	motorX1->setPower(power_out[0]);
	motorX2->setPower(power_out[1]);
	motorY1->setPower(power_out[2]);
	motorY2->setPower(power_out[3]);
}
