#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP

#include <QObject>
#include "CopterCtrl.hpp"

class CopterCtrl;

class Accelerometer : public QObject
{
	Q_OBJECT
public:
	explicit Accelerometer(const QString inputPath, CopterCtrl* copterCtrl, QObject *parent = 0);
	~Accelerometer();
	
	void adjustZeroAxis();
	QVector3D zeroAxis() { return m_zeroAxis; }

signals:
	void accelerometerRead(QVector3D val);
	void zeroAxisChanged(QVector3D val);

public slots:
	void onRead();

private:
	QVector3D filterAxis(QVector3D axis);
	QVector3D filterMean(QVector3D axis);
	QVector3D filterKalman(QVector3D axis);
	QVector3D filterLinear(QVector3D axis);
	QVector3D filterLinearAlt(QVector3D axis);

	QVector3D m_kalmanOpt;
	QVector3D m_linearOpt[3];

	float minVal, maxVal;
	int m_adjustCounter;
	int m_inputFd;
	QVector3D m_zeroAxis;
	QVector3D m_curAxis;
	QVector3D m_prevAxis[5];
	int m_meanCounter;
	int m_linearCounter;
	CopterCtrl* m_copterCtrl;
	QSocketNotifier* m_inputNotifier;
};

#endif // ACCELEROMETER_HPP
