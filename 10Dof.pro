QT += core
QT += gui
QT += widgets
Qt += QtNetwork

TARGET = 10Dof
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS


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
    utilities.cpp \
    MotorController_BST7960.cpp \
    MotorController_L298.cpp

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
    utilities.h \
    MotorController_BST7960.h \
    MotorController_L298.h

LIBS += -lpigpiod_if2 # To include libpigpiod_if2.so from /usr/local/lib
LIBS += -lQt5Network

FORMS    +=

RESOURCES += \
    shaders.qrc \
    textures.qrc

DISTFILES += \
    plot.png \
    Inertial-Frame.png \
    .gitignore
