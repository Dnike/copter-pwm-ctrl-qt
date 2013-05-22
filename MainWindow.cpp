#include <QStatusBar>

#include "MainWindow.hpp"

MainWindow::MainWindow(CopterCtrl *copterCtrl, QWidget* _parent) :
	QMainWindow(_parent),
	m_copterCtrl(copterCtrl),
	m_ui(new Ui::MainWindow())
{
	m_ui->setupUi(this);

	// TODO: use flight control signals
//	connect(m_copterCtrl, SIGNAL(accelerometerRead(QVector3D)), this, SLOT(onAccelerometerRead(QVector3D)));
//	connect(m_copterCtrl, SIGNAL(motorPowerChanged(CopterCtrl::Motor,float)),
//					this, SLOT(onMotorPowerChange(CopterCtrl::Motor,float)));

	// customize status bar appearance
	QFont statusBarFont = statusBar()->font();
	statusBarFont.setPointSize(5);
	statusBar()->setFont(statusBarFont);

	showFullScreen();
}

void MainWindow::onAccelerometerRead(QVector3D val)
{
	m_ui->cur_accel_x->setText(QString::number(static_cast<int>(val.x())));
	m_ui->cur_accel_y->setText(QString::number(static_cast<int>(val.y())));
	m_ui->cur_accel_z->setText(QString::number(static_cast<int>(val.z())));
}

void MainWindow::onMotorPowerChange(FlightControl::Motor motor, float power)
{
	QLCDNumber* lcd;
	switch (motor) {
		case FlightControl::MotorX1: lcd = m_ui->motor_x1; break;
		case FlightControl::MotorX2: lcd = m_ui->motor_x2; break;
		case FlightControl::MotorY1: lcd = m_ui->motor_y1; break;
		case FlightControl::MotorY2: lcd = m_ui->motor_y2; break;
		case FlightControl::MotorAll: lcd = m_ui->motor_all; break;
	}
	lcd->display(power);
}


