#include <QStyleFactory>
#include <QDebug>
#include <QStringList>
#include <QWSServer>

#include "MainWindow.hpp"
#include "CopterCtrl.hpp"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

#ifdef Q_WS_QWS
	QWSServer::setCursorVisible( false );
#endif
	a.setStyle(QStyleFactory::create("Cleanlooks"));

	// main control class
	CopterCtrl* ctrl = new CopterCtrl();
	
	// graphics on board screen (no use for now, just a template for future)
	if (!ctrl->getSettings()->value("NoGraphics").toBool()) {
		MainWindow* w = new MainWindow(ctrl);
		w->show();
	}
	return a.exec();
}

