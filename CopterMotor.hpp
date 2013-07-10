#pragma once

#include <QLCDNumber>
#include <QFile>
#include <QSettings>

class CopterMotor : public QObject
{
	Q_OBJECT
public:
	CopterMotor(int _powerZero, int _powerMin, int _powerMax, const QString& _ctrlPath);
	~CopterMotor();

	void setPower(int _power);
	int getPower() const { return power; }

signals:
	void powerChanged(float power);

public slots:
	void emergencyStop();

protected:
	QFile       ctrlFile;
	float      delta;
	int powerMax, powerMin, powerZero; // real power, to write to ctrlFile
	void invoke(int _power);
	int power;
};
