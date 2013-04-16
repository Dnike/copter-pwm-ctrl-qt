#include "gyro.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>

#include <QTime>

Gyro::Gyro(const QString &inputPath, CopterCtrl *copterCtrl, QObject *parent) :
	m_inputFd(-1),
	m_inputNotifier(0),
	m_copterCtrl(copterCtrl),
	m_curVal(),
	m_lastVal()
{
	m_inputFd = ::open(inputPath.toLatin1().data(), O_SYNC, O_RDONLY);
	if (m_inputFd == -1)
		qDebug() << "Cannot open accelerometer input file " << inputPath << ", reason: " << errno;

	m_inputNotifier = new QSocketNotifier(m_inputFd, QSocketNotifier::Read, this);
	connect(m_inputNotifier, SIGNAL(activated(int)), this, SLOT(onRead()));
	m_inputNotifier->setEnabled(true);
}


void Gyro::onRead()
{
	struct input_event evt;

	if (read(m_inputFd, reinterpret_cast<char*>(&evt), sizeof(evt)) != sizeof(evt))
	{
		qDebug() << "Incomplete gyro data read";
		return;
	}

	if (evt.type != EV_ABS)
	{
		if (evt.type != EV_SYN)
			qDebug() << "Input event type is not EV_ABS or EV_SYN: " << evt.type;
		else {
			m_lastVal = m_curVal;
			emit dataRead(m_lastVal);
		}
		return;
	}

	switch (evt.code)
	{
		case ABS_X:
			m_curVal.setX(evt.value);
			break;
		case ABS_Y:
			m_curVal.setY(evt.value);
			break;
		case ABS_Z:
			m_curVal.setZ(evt.value);
			break;
	}
}
