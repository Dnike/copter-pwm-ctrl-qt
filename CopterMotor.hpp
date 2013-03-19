#pragma once

#include <QLCDNumber>
#include <QFile>
#include <QSettings>

class CopterMotor : public QObject
{
	Q_OBJECT
public:
	CopterMotor(int powerMin, int powerMax, const QString& _ctrlPath);
	~CopterMotor();

	void setPower(int _power);
	int power() const { return m_power; }

signals:
	void powerChanged(float power);

public slots:
	void emergencyStop();

protected:
	QFile       m_ctrlFile;
	float      m_delta;
	int m_powerMax, m_powerMin; // real power, to write to ctrlFile
	void invoke(int _power);
	int m_power;
};
