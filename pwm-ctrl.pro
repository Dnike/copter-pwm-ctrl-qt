TEMPLATE =      app

CONFIG +=       debug_and_release \
                warn_on \
                copy_dir_files

debug:CONFIG += console

CONFIG -=       warn_off

QT +=           network

contains($$[QT_VERSION_MAJOR],5) {
    QT += widgets
}

TARGET =        copter-pwm-ctrl-qt

SOURCES +=      \
    CopterMotor.cpp \
    CopterCtrl.cpp \
    CopterAxis.cpp \
    MainWindow.cpp \
    Main.cpp \
    accelerometer.cpp \
    gyro.cpp \
    flightcontrol.cpp

HEADERS +=      \
    CopterMotor.hpp \
    CopterCtrl.hpp \
    CopterAxis.hpp \
    MainWindow.hpp \
    accelerometer.hpp \
    gyro.hpp \
    flightcontrol.hpp

FORMS += \       
    MainWindow.ui

QMAKE_CXXFLAGS += -std=c++0x
QMAKE_CXXFLAGS_WARN_ON = -Wno-reorder

unix {
  target.path = $$[INSTALL_ROOT]/bin
  INSTALLS +=   target
}
