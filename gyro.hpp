#ifndef GYRO_HPP
#define GYRO_HPP

#include <QObject>
#include "CopterCtrl.hpp"

class Gyro : public QObject
{
	Q_OBJECT
public:
	explicit Gyro(const QString& inputPath, CopterCtrl* copterCtrl, QObject *parent = 0);
	
	QVector3D getLastVal() { return m_lastVal; }

signals:
	void dataRead(QVector3D val);

public slots:

protected slots:
	void onRead();

private:
	QVector3D m_curVal;
	QVector3D m_lastVal;

	int m_inputFd;
	QSocketNotifier* m_inputNotifier;

	CopterCtrl* m_copterCtrl;
};

#endif // GYRO_HPP
