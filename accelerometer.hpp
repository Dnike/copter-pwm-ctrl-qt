#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP

#include <QObject>
#include "CopterCtrl.hpp"

class Accelerometer : public QObject
{
	Q_OBJECT
public:
	explicit Accelerometer(const QString inputPath, CopterCtrl* copterCtrl, QObject *parent = 0);
	~Accelerometer();
	
	void adjustZeroAxis();
	QVector3D getLastVal() { return m_lastAxis; }

signals:
	void dataRead(QVector3D val);

protected slots:
	void onRead();

private:
	QVector3D filterAxis(QVector3D axis);
	QVector3D filterMean(QVector3D axis);
	QVector3D filterKalman(QVector3D axis);
	QVector3D filterLinear(QVector3D axis);
	QVector3D filterLinearAlt(QVector3D axis);

	QVector3D m_kalmanOpt;
	QVector<QVector3D> m_linearOpt;

	QVector3D m_zeroAxis;
	QVector3D m_curAxis;
	QVector3D m_lastAxis;
	QVector<QVector3D> m_prevAxis;
	int m_meanCounter;
	int m_linearCounter;
	CopterCtrl* m_copterCtrl;

	int m_inputFd;
	QSocketNotifier* m_inputNotifier;
};

#endif // ACCELEROMETER_HPP
