#-------------------------------------------------
#
# Project created by QtCreator 2020-04-24T15:57:31
#
#-------------------------------------------------

QT       += core
QT       += gui
QT       += widgets


TARGET = 10Dof
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    geometryengine.cpp \
    HMC5883L.cpp \
    MadgwickAHRS.cpp \
    MahonyAHRS.cpp \
    GLwidget.cpp \
    ADXL345.cpp \
    ITG3200.cpp \
    axesdialog.cpp \
    AxisFrame.cpp \
    AxisLimits.cpp \
    DataSetProperties.cpp \
    datastream2d.cpp \
    plot2d.cpp \
    plotpropertiesdlg.cpp \
    PID_v1.cpp \
    utilities.cpp

HEADERS  += mainwindow.h \
    geometryengine.h \
    HMC5883L.h \
    MadgwickAHRS.h \
    MahonyAHRS.h \
    GLwidget.h \
    ADXL345.h \
    ITG3200.h \
    axesdialog.h \
    AxisFrame.h \
    AxisLimits.h \
    DataSetProperties.h \
    datastream2d.h \
    plot2d.h \
    plotpropertiesdlg.h \
    PID_v1.h \
    utilities.h

FORMS    +=

RESOURCES += \
    shaders.qrc \
    textures.qrc

DISTFILES += \
    plot.png \
    Inertial-Frame.png
