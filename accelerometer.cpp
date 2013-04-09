#include "accelerometer.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>

#include <QTime>

Accelerometer::Accelerometer(const QString inputPath, CopterCtrl* copterCtrl, QObject *parent) :
	QObject(parent),
	m_inputFd(-1),
	m_inputNotifier(0),
	m_adjustCounter(0),
	m_copterCtrl(copterCtrl),
	m_zeroAxis(),
	m_curAxis(),
	m_meanCounter(0),
	m_linearCounter(0),
	m_kalmanOpt()
{
	// TODO: write with vectors
	for (int i = 0; i < 5; ++i) m_prevAxis[i] = QVector3D();
	for (int i = 0; i < 3; ++i) m_linearOpt[i] = QVector3D();
	m_inputFd = ::open(inputPath.toLatin1().data(), O_SYNC, O_RDONLY);
	if (m_inputFd == -1)
		qDebug() << "Cannot open accelerometer input file " << inputPath << ", reason: " << errno;

	m_inputNotifier = new QSocketNotifier(m_inputFd, QSocketNotifier::Read, this);
	connect(m_inputNotifier, SIGNAL(activated(int)), this, SLOT(onRead()));
	m_inputNotifier->setEnabled(true);
}

Accelerometer::~Accelerometer()
{
}

void Accelerometer::onRead()
{
	struct input_event evt;

	if (read(m_inputFd, reinterpret_cast<char*>(&evt), sizeof(evt)) != sizeof(evt))
	{
		qDebug() << "Incomplete accelerometer data read";
		return;
	}

	if (evt.type != EV_ABS)
	{
		if (evt.type != EV_SYN)
			qDebug() << "Input event type is not EV_ABS or EV_SYN: " << evt.type;
		else {
			if (m_copterCtrl->state() == CopterCtrl::ADJUSTING_ACCEL) {
				m_zeroAxis = (m_zeroAxis * m_adjustCounter + m_curAxis) / (m_adjustCounter + 1);
				++m_adjustCounter;
				emit zeroAxisChanged(m_zeroAxis);
			}
			emit accelerometerRead(filterAxis(m_curAxis - m_zeroAxis));
		}
		return;
	}

	switch (evt.code)
	{
		case ABS_X:
			m_curAxis.setX(evt.value);
			break;
		case ABS_Y:
			m_curAxis.setY(evt.value);
			break;
		case ABS_Z:
			m_curAxis.setZ(evt.value);
			break;
	}
}

QVector3D Accelerometer::filterAxis(QVector3D axis)
{
	QVector3D res;
	switch (m_copterCtrl->getSettings()->value("FilterMethod").toInt()) {
		case 0: res = filterMean(axis); break;
		case 1: res = filterKalman(axis); break;
		case 2: res = filterLinear(axis); break;
		case 3: res = filterLinearAlt(axis); break;
		case 4: res = filterKalman(filterLinear(axis)); break;
		case 5: res = filterLinear(filterKalman(axis)); break;
		case 6: res = filterKalman(filterLinearAlt(axis)); break;
		case 7: res = filterLinearAlt(filterKalman(axis)); break;
		default: res = filterKalman(axis); break;
	}
	return res;
}

QVector3D Accelerometer::filterMean(QVector3D axis)
{
	QVector3D countedAxis;
	m_prevAxis[m_meanCounter] = axis;
	m_meanCounter = (m_meanCounter + 1) % 5;
	for (int i = 0; i < 5; ++i) countedAxis = countedAxis + m_prevAxis[i] / 5;
	return countedAxis;
}

QVector3D Accelerometer::filterKalman(QVector3D axis)
{
	float k = m_copterCtrl->getSettings()->value("KalmanK").toFloat();
	m_kalmanOpt = m_kalmanOpt * k + axis * (1 - k);
	return m_kalmanOpt;
}

QVector3D Accelerometer::filterLinear(QVector3D axis)
{
	m_linearCounter = (m_linearCounter + 1) % 3;
	m_linearOpt[m_linearCounter] = (axis +
																	m_linearOpt[(m_linearCounter + 1) % 3] * 3 +
																 m_linearOpt[(m_linearCounter + 2) % 3] * 3 +
																 m_linearOpt[m_linearCounter]) / 8;
	return m_linearOpt[m_linearCounter];
}

QVector3D Accelerometer::filterLinearAlt(QVector3D axis)
{
	m_linearCounter = (m_linearCounter + 1) % 2;
	m_linearOpt[m_linearCounter] = (axis +
																	m_linearOpt[(m_linearCounter + 1) % 2] * 2 +
																 m_linearOpt[m_linearCounter]) / 4;
	return m_linearOpt[m_linearCounter];
}

void Accelerometer::adjustZeroAxis()
{
	m_adjustCounter = 0;
	m_zeroAxis.setX(0);
	m_zeroAxis.setY(0);
	m_zeroAxis.setZ(0);
}

