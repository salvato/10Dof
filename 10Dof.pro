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
    ITG3200.cpp

HEADERS  += mainwindow.h \
    geometryengine.h \
    HMC5883L.h \
    MadgwickAHRS.h \
    MahonyAHRS.h \
    GLwidget.h \
    ADXL345.h \
    ITG3200.h

FORMS    += mainwindow.ui

RESOURCES += \
    shaders.qrc \
    textures.qrc
