#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "plot2d.h"
#include <QtWidgets>
#include <QDebug>
#include <QThread>
#include <GLwidget.h>
#include <QPushButton>
#include "PID_v1.h"
#include "MotorController_L298.h"
#include "MotorController_BST7960.h"

#include <cmath>

// Hardware Connections:
//
// DFRobot 10DOF :
//      SDA on BCM 2:    pin 3 in the 40 pins GPIO connector
//      SDL on BCM 3:    pin 5 in the 40 pins GPIO connector
//      Vcc on 5V Power: pin 4 in the 40 pins GPIO connector
//      GND on GND:      pin 6 in the 40 pins GPIO connector
//
// For Raspberry Pi GPIO pin numbering see
// https://pinout.xyz/
//
// +5V on pins 2 or 4 in the 40 pin GPIO connector.
// GND on pins 6, 9, 14, 20, 25, 30, 34 or 39
// in the 40 pin GPIO connector.

// For L298 MotorController
//#define PWM1_PIN  12 // on BCM12: Pin 32 in the 40 pin GPIO connector.
//#define M1IN1_PIN 17 // on BCM17: Pin 11 in the 40 pin GPIO connector.
//#define M1IN2_PIN 27 // on BCM27: Pin 13 in the 40 pin GPIO connector.
//#define PWM2_PIN  13 // on BCM13: Pin 33 in the 40 pin GPIO connector.
//#define M2IN1_PIN 22 // on BCM22: Pin 15 in the 40 pin GPIO connector.
//#define M2IN2_PIN 23 // on BCM23: Pin 16 in the 40 pin GPIO connector.

// For BST760 MotorController
//uint32_t pwm1Up, uint32_t pwm1Low,
//uint32_t pwm2Up, uint32_t pwm2Low,

#define PWM1UP_PIN  12 // on BCM12: Pin 32 in the 40 pin GPIO connector.
#define PWM1LOW_PIN 13 // on BCM13: Pin 33 in the 40 pin GPIO connector.
#define PWM2UP_PIN  22 // on BCM22: Pin 15 in the 40 pin GPIO connector.
#define PWM2LOW_PIN 23 // on BCM23: Pin 16 in the 40 pin GPIO connector.

#define MIN_ABS_SPEED 0

//==============================================================
// Informations for connecting servos:
//
// Samwa servo pinout:
//      1) PWM Signal
//      2) GND
//      3) +5V
//==============================================================

// I2C Addresses :
//      0x1E     HMC5883L_Address        (Magnetometer)
//      0x53     ADXL345_ADDR_ALT_LOW    (Accelerometer)
//      0x68     ITG3200_ADDR_AD0_LOW    (Gyroscope)
//      0x77     BMP085_ADDRESS          (Pressure and Temperature)


MainWindow::MainWindow()
    : pGLWidget(nullptr)
    , pPlotVal(nullptr)
    , pPid(nullptr)
    , bAccCalInProgress(false)
    , bGyroCalInProgress(false)
    , bMagCalInProgress(false)
    , bShowPidInProgress(false)
{
    buttonStartStop       = new QPushButton("Start",     this);
    buttonAccCalibration  = new QPushButton("Acc. Cal.", this);
    buttonGyroCalibration = new QPushButton("Gyro Cal.", this);
    buttonMagCalibration  = new QPushButton("Mag. Cal.", this);
    buttonShowPidOutput   = new QPushButton("Show PID",  this);

    buttonStartStop->setEnabled(true);
    buttonAccCalibration->setEnabled(false);
    buttonGyroCalibration->setEnabled(false);
    buttonMagCalibration->setEnabled(false);
    buttonShowPidOutput->setEnabled(false);

    connect(buttonStartStop, SIGNAL(clicked()),
            this, SLOT(onStartStopPushed()));
    connect(buttonAccCalibration, SIGNAL(clicked()),
            this, SLOT(onStartAccCalibration()));
    connect(buttonGyroCalibration, SIGNAL(clicked()),
            this, SLOT(onStartGyroCalibration()));
    connect(buttonMagCalibration, SIGNAL(clicked()),
            this, SLOT(onStartMagCalibration()));
    connect(buttonShowPidOutput, SIGNAL(clicked(bool)),
            this, SLOT(onShowPidOutput()));

    pGLWidget = new GLWidget(this);

    pPlotVal = new Plot2D(this, "Plot");

    pPlotVal->NewDataSet(1, 1, QColor(255,   0,   0), Plot2D::ipoint, "X");
    pPlotVal->NewDataSet(2, 1, QColor(  0, 255,   0), Plot2D::ipoint, "Y");
    pPlotVal->NewDataSet(3, 1, QColor(  0,   0, 255), Plot2D::ipoint, "Z");
    pPlotVal->NewDataSet(4, 1, QColor(255, 255, 255), Plot2D::ipoint, "PID");

    pPlotVal->SetShowTitle(1, true);
    pPlotVal->SetShowTitle(2, true);
    pPlotVal->SetShowTitle(3, true);
    pPlotVal->SetShowTitle(4, true);

    pPlotVal->SetLimits(-1.0, 1.0, -1.0, 1.0, true, true, false, false);

    initLayout();

    pAcc  = new ADXL345(); // init ADXL345
    pAcc->init(ACC_ADDR);

    pGyro = new ITG3200(); // init ITG3200
    pGyro->init(ITG3200_DEF_ADDR);
    QThread::msleep(3000);
    pGyro->zeroCalibrate(1024, 5); // calibrate the ITG3200

    pMagn = new HMC5883L();// init HMC5883L
    // The magnitude of the Earth's magnetic field at its surface
    // ranges from 250 to 650 milli Gauss.
    int16_t error = pMagn->SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        qDebug() << pMagn->GetErrorText(error);
        //exit(EXIT_FAILURE);
    }
    error = pMagn->SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0)  {
        qDebug() << pMagn->GetErrorText(error);
        //exit(EXIT_FAILURE);
    }

    //bmp085Calibration(); // init barometric pressure sensor

    pMadgwick = new Madgwick();

    motorSpeedFactorLeft  = 0.6;
    motorSpeedFactorRight = 0.5;
//    pMotorController = new MotorController_L298(PWM1_PIN, M1IN1_PIN, M1IN2_PIN,
//                                                PWM2_PIN, M2IN1_PIN, M2IN2_PIN,
//                                                motorSpeedFactorLeft, motorSpeedFactorRight);
    pMotorController = new MotorController_BST7960(PWM1UP_PIN, PWM1LOW_PIN,
                                                   PWM2UP_PIN, PWM2LOW_PIN,
                                                   motorSpeedFactorLeft, motorSpeedFactorRight);
    double originalSetpoint = 0.0;
    setpoint = originalSetpoint;
    movingAngleOffset = 0.1;
    moveState = 0; // 0 = balance; 1 = back; 2 = forth
    Kp =  1.0;//50.0;
    Kd =  0.0;//1.4;
    Ki =  0.0;//60.0;
    pPid = new PID(Kp, Ki, Kd, ControllerDirection);

    pPid->SetMode(AUTOMATIC);
    pPid->SetSampleTime(100); // in ms
    pPid->SetOutputLimits(-255, 255);

    sampleFrequency = 300;
    pMadgwick->begin(sampleFrequency);

    // Consider to change to QBasicTimer that it's faster than QTimer
    loopTimer.setTimerType(Qt::PreciseTimer);
    connect(&loopTimer, SIGNAL(timeout()),
            this, SLOT(onLoopTimeElapsed()));

    while(!pAcc->getInterruptSource(7)) {}
    pAcc->get_Gxyz(&values[0]);
    while(!pGyro->isRawDataReadyOn()) {}
    pGyro->readGyro(&values[3]);
    while(!pMagn->isDataReady()) {}
    pMagn->ReadScaledAxis(&values[6]);

    for(int i=0; i< 10000; i++) {
        pMadgwick->update(values[3], values[4], values[5],
                values[0], values[1], values[2],
                values[6], values[7], values[8]);
    }
    pMadgwick->getRotation(&q0, &q1, &q2, &q3);
    pGLWidget->setRotation(q0, q1, q2, q3);
}


MainWindow::~MainWindow() {
    if(pGLWidget) delete pGLWidget;
    pGLWidget = nullptr;
}


void
MainWindow::closeEvent(QCloseEvent *event) {
    Q_UNUSED(event)
    loopTimer.stop();
    if(pGLWidget) delete pGLWidget;
    pGLWidget = nullptr;
    if(pPlotVal) delete pPlotVal;
    pPlotVal = nullptr;
}



void
MainWindow::initLayout() {
    QHBoxLayout *firstButtonRow = new QHBoxLayout;
    firstButtonRow->addWidget(buttonStartStop);
    firstButtonRow->addWidget(buttonAccCalibration);
    firstButtonRow->addWidget(buttonGyroCalibration);
    firstButtonRow->addWidget(buttonMagCalibration);
    firstButtonRow->addWidget(buttonShowPidOutput);

    QHBoxLayout *firstRow = new QHBoxLayout;
    firstRow->addWidget(pGLWidget);
    firstRow->addWidget(pPlotVal);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(firstRow);
    mainLayout->addLayout(firstButtonRow);

    setLayout(mainLayout);
}


void
MainWindow::keyPressEvent(QKeyEvent *e) {
  if(e->key() == Qt::Key_Escape)
    close();
  else
    QWidget::keyPressEvent(e);
}


void
MainWindow::onStartStopPushed() {
    if(bRunInProgress) {
        loopTimer.stop();
        bRunInProgress = false;
        buttonStartStop->setText("Start");
        buttonAccCalibration->setEnabled(false);
        buttonGyroCalibration->setEnabled(false);
        buttonMagCalibration->setEnabled(false);
        buttonShowPidOutput->setEnabled(false);
    }
    else {
        bRunInProgress = true;
        buttonStartStop->setText("Stop");
        buttonAccCalibration->setEnabled(true);
        buttonGyroCalibration->setEnabled(true);
        buttonMagCalibration->setEnabled(true);
        buttonShowPidOutput->setEnabled(true);
        lastUpdate = micros();
        now = micros();
        loopTimer.start(int32_t(1000.0/sampleFrequency+0.5));
    }
}


void
MainWindow::onStartAccCalibration() {
    if(bAccCalInProgress) {
        buttonAccCalibration->setText("Acc. Cal.");
        bAccCalInProgress = false;
        buttonGyroCalibration->setEnabled(true);
        buttonMagCalibration->setEnabled(true);
        buttonShowPidOutput->setEnabled(true);
    }
    else {
        pPlotVal->ClearDataSet(1);
        pPlotVal->ClearDataSet(2);
        pPlotVal->ClearDataSet(3);

        pPlotVal->SetShowDataSet(1, true);
        pPlotVal->SetShowDataSet(2, true);
        pPlotVal->SetShowDataSet(3, true);
        pPlotVal->SetShowDataSet(4, false);

        bGyroCalInProgress = false;
        bMagCalInProgress = false;
        bShowPidInProgress = false;
        bAccCalInProgress = true;
        buttonAccCalibration->setText("Stop Cal.");
        buttonGyroCalibration->setDisabled(true);
        buttonMagCalibration->setDisabled(true);
        buttonShowPidOutput->setDisabled(true);
        avgX = avgY = avgZ = 0.0;
        nAvg = 10;
        nCurr = 0;
        t0 = micros();
    }
}


void
MainWindow::onStartGyroCalibration() {
    if(bGyroCalInProgress) {
        buttonGyroCalibration->setText("Gyro Cal.");
        bGyroCalInProgress = false;
        buttonAccCalibration->setEnabled(true);
        buttonMagCalibration->setEnabled(true);
        buttonShowPidOutput->setEnabled(true);
    }
    else {
        pPlotVal->ClearDataSet(1);
        pPlotVal->ClearDataSet(2);
        pPlotVal->ClearDataSet(3);

        pPlotVal->SetShowDataSet(1, true);
        pPlotVal->SetShowDataSet(2, true);
        pPlotVal->SetShowDataSet(3, true);
        pPlotVal->SetShowDataSet(4, false);

        bShowPidInProgress = false;
        bMagCalInProgress = false;
        bAccCalInProgress = false;
        bGyroCalInProgress = true;
        buttonGyroCalibration->setText("Stop Cal.");
        buttonAccCalibration->setDisabled(true);
        buttonMagCalibration->setDisabled(true);
        buttonShowPidOutput->setDisabled(true);
        angleX = 0.0;
        angleY = 0.0;
        angleZ = 0.0;
        t0 = micros();
    }
}


void
MainWindow::onStartMagCalibration() {
    if(bMagCalInProgress) {
        bMagCalInProgress = false;
        buttonMagCalibration->setText("Mag. Cal.");
        buttonAccCalibration->setEnabled(true);
        buttonGyroCalibration->setEnabled(true);
        buttonShowPidOutput->setEnabled(true);
    }
    else {
        pPlotVal->ClearDataSet(1);
        pPlotVal->ClearDataSet(2);
        pPlotVal->ClearDataSet(3);

        pPlotVal->SetShowDataSet(1, true);
        pPlotVal->SetShowDataSet(2, true);
        pPlotVal->SetShowDataSet(3, true);
        pPlotVal->SetShowDataSet(4, false);

        buttonMagCalibration->setText("Stop Cal.");
        bAccCalInProgress = false;
        bGyroCalInProgress = false;
        bMagCalInProgress = true;
        bShowPidInProgress = false;
        buttonAccCalibration->setDisabled(true);
        buttonGyroCalibration->setDisabled(true);
        buttonShowPidOutput->setDisabled(true);
        t0 = micros();
    }
}


void
MainWindow::onShowPidOutput() {
    if(bShowPidInProgress) {
        bShowPidInProgress = false;
        buttonShowPidOutput->setText("Show PID");
        buttonAccCalibration->setEnabled(true);
        buttonGyroCalibration->setEnabled(true);
        buttonMagCalibration->setEnabled(true);
    }
    else {
        pPlotVal->ClearDataSet(4);

        pPlotVal->SetShowDataSet(1, false);
        pPlotVal->SetShowDataSet(2, false);
        pPlotVal->SetShowDataSet(3, false);
        pPlotVal->SetShowDataSet(4, true);

        buttonShowPidOutput->setText("Hide Pid Out");
        bAccCalInProgress  = false;
        bGyroCalInProgress = false;
        bMagCalInProgress  = false;
        bShowPidInProgress = true;
        buttonAccCalibration->setDisabled(true);
        buttonGyroCalibration->setDisabled(true);
        buttonMagCalibration->setDisabled(true);
        t0 = micros();
    }
}


void
MainWindow::onLoopTimeElapsed() {
    //==================================================================
    //  !!! Attention !!!
    //==================================================================
    // Reasonable convergence can be achieved in two or three iterations
    // meaning that we should operate this sensor fusion filter at a
    // rate two or three times the output data rate of the sensor.
    //
    // Deliver sensor values at the Madgwick algorithm
    // in the expected format which is rad/s, m/s² and mG (milliGauss)
    //==================================================================

    if(pAcc->getInterruptSource(7)) {
        pAcc->get_Gxyz(&values[0]);
        if(bAccCalInProgress) {
            double x = (micros()-t0)/1000000.0;
            avgX += double(values[0]);
            avgY += double(values[1]);
            avgZ += double(values[2]);
            nCurr++;
            if(nCurr == nAvg) {
                avgX /= double(nAvg);
                avgY /= double(nAvg);
                avgZ /= double(nAvg);
                nCurr = 0;
                pPlotVal->NewPoint(1, x, avgX);
                pPlotVal->NewPoint(2, x, avgY);
                pPlotVal->NewPoint(3, x, avgZ);
            }
            avgX = avgY = avgZ = 0.0;
        }
    }

    if(pGyro->isRawDataReadyOn()) {
        pGyro->readGyro(&values[3]);
        if(bGyroCalInProgress) {
            double x = micros();
            double d = lastUpdate - x;
            x = (x-t0)/1000000.0;
            d /= 1000000.0;
            angleX += double(values[3]) * d;
            angleY += double(values[4]) * d;
            angleZ += double(values[5]) * d;
            pPlotVal->NewPoint(1, x, angleX);
            pPlotVal->NewPoint(2, x, angleY);
            pPlotVal->NewPoint(3, x, angleZ);
        }
    }

    if(pMagn->isDataReady()) {
        pMagn->ReadScaledAxis(&values[6]);
        if(bMagCalInProgress) {
            pPlotVal->NewPoint(1, double(values[6]), double(values[7]));
            pPlotVal->NewPoint(2, double(values[7]), double(values[8]));
            pPlotVal->NewPoint(3, double(values[8]), double(values[6]));
        }
    }
    now = micros();
    delta = float(now-lastUpdate)/1000000.f;
    pMadgwick->setInvFreq(delta);
    lastUpdate = now;
    pMadgwick->update(values[3], values[4], values[5],
                      values[0], values[1], values[2],
                      values[6], values[7], values[8]);

//    pMadgwick->updateIMU(values[3], values[4], values[5],
//                         values[0], values[1], values[2]);

    nUpdate++;
    nUpdate = nUpdate % 10;
    if(!nUpdate) {
        pMadgwick->getRotation(&q0, &q1, &q2, &q3);
        pGLWidget->setRotation(q0, q1, q2, q3);
        pGLWidget->update();
        pPlotVal->UpdatePlot();
    }
    input = pMadgwick->getPitch();
    output = pPid->Compute(input, setpoint);
    pMotorController->move(output, MIN_ABS_SPEED);
    if(bShowPidInProgress)
        pPlotVal->NewPoint(4, double(now-t0)/1000000.0, output);
}
