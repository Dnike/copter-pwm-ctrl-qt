#include <QStatusBar>

#include "MainWindow.hpp"

MainWindow::MainWindow(CopterCtrl *copterCtrl, QWidget* _parent) :
	QMainWindow(_parent),
	m_copterCtrl(copterCtrl),
	m_ui(new Ui::MainWindow())
{
	m_settings = m_copterCtrl->getSettings();
	m_ui->setupUi(this);

	connect(m_copterCtrl, SIGNAL(stateChanged(CopterState)), this, SLOT(onStateChange()));
	connect(m_copterCtrl, SIGNAL(accelerometerRead(QVector3D)), this, SLOT(onAccelerometerRead(QVector3D)));
	connect(m_copterCtrl, SIGNAL(motorPowerChanged(CopterCtrl::Motor,float)),
					this, SLOT(onMotorPowerChange(CopterCtrl::Motor,float)));

	// customize status bar appearance
	QFont statusBarFont = statusBar()->font();
	statusBarFont.setPointSize(5);
	statusBar()->setFont(statusBarFont);
	statusBar()->showMessage(m_copterCtrl->stateString());

	// temporary, move to main
	m_copterCtrl->adjustPower(0);

	showFullScreen();
}

void MainWindow::onStateChange()
{
	statusBar()->showMessage(m_copterCtrl->stateString());
}

void MainWindow::onAccelerometerRead(QVector3D val)
{
	m_ui->cur_accel_x->setText(QString::number(static_cast<int>(val.x())));
	m_ui->cur_accel_y->setText(QString::number(static_cast<int>(val.y())));
	m_ui->cur_accel_z->setText(QString::number(static_cast<int>(val.z())));
}

void MainWindow::onMotorPowerChange(CopterCtrl::Motor motor, float power)
{
	QLCDNumber* lcd;
	switch (motor) {
		case CopterCtrl::MotorX1: lcd = m_ui->motor_x1; break;
		case CopterCtrl::MotorX2: lcd = m_ui->motor_x2; break;
		case CopterCtrl::MotorY1: lcd = m_ui->motor_y1; break;
		case CopterCtrl::MotorY2: lcd = m_ui->motor_y2; break;
		case CopterCtrl::MotorAll: lcd = m_ui->motor_all; break;
	}

	QPalette palette = lcd->palette();
	QColor bg = palette.color(QPalette::Disabled, lcd->backgroundRole());
	float powerMax = m_settings->value("PowerMax").toFloat();
	float powerMin = m_settings->value("PowerMin").toFloat();
	float pwrSat = 1.0 - static_cast<float>((power - powerMin) / (2 * powerMax - powerMin));
	bg.setBlue(bg.blue()   * pwrSat);
	bg.setGreen(bg.green() * pwrSat + 0xff * (1.0 - pwrSat));
	bg.setRed(bg.red()     * pwrSat);
	palette.setColor(QPalette::Normal, lcd->backgroundRole(), bg);
	palette.setColor(QPalette::Active, lcd->backgroundRole(), bg);
	palette.setColor(QPalette::Inactive, lcd->backgroundRole(), bg);
	lcd->setPalette(palette);
	lcd->display(power);
}


