#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QThread>

#include <cmath>


// Addresses :
// 0x1E     HMC5883L_Address        (Magnetometer)
// 0x53     ADXL345_ADDR_ALT_LOW    (Accelerometer)
// 0x68     ITG3200_ADDR_AD0_LOW    (Gyroscope)
// 0x77     BMP085_ADDRESS          (Pressure and Temperature)


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , pGLWidget(nullptr)
{
    ui->setupUi(this);

    pGLWidget = new GLWidget(nullptr);
    pGLWidget->show();

    pAcc  = new ADXL345(); // init ADXL345
    pAcc->init(FIMU_ACC_ADDR);

    pGyro = new ITG3200(); // init ITG3200
    pGyro->init(FIMU_ITG3200_DEF_ADDR);
    QThread::msleep(1000);
    pGyro->zeroCalibrate(128, 5); // calibrate the ITG3200

    pMagn = new HMC5883L();// init HMC5883L
    // The magnitude of the Earth's magnetic field at its surface
    // ranges from 250 to 650 milli Gauss.
    int16_t error = pMagn->SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        qDebug() << pMagn->GetErrorText(error);
        exit(EXIT_FAILURE);
    }
    error = pMagn->SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0)  {
        qDebug() << pMagn->GetErrorText(error);
        exit(EXIT_FAILURE);
    }

    //bmp085Calibration(); // init barometric pressure sensor

    pMadgwick = new Madgwick();

    sampleFrequency = 1000;
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

    lastUpdate = micros();
    now = micros();
    loopTimer.start(int32_t(1000.0/sampleFrequency+0.5));
}


MainWindow::~MainWindow() {
    if(pGLWidget) delete pGLWidget;
    pGLWidget = nullptr;
    delete ui;
}


void
MainWindow::closeEvent(QCloseEvent *event) {
    Q_UNUSED(event)
    if(pGLWidget) delete pGLWidget;
    pGLWidget = nullptr;
}


void
MainWindow::onLoopTimeElapsed() {
    //
    //  !!! Warning !!!
    //
    // Reasonable convergence can be achieved in two or three iterations
    // meaning that we would like this sensor fusion filter to operate
    // a rate two or three times the output data rate of the sensor.
    //
    // Within the Madgwick algorithm deliver your values
    // in the expected format which is rad/s and m/s²

    if(pAcc->getInterruptSource(7))
        pAcc->get_Gxyz(&values[0]);
    if(pGyro->isRawDataReadyOn())
        pGyro->readGyro(&values[3]);
    if(pMagn->isDataReady())
        pMagn->ReadScaledAxis(&values[6]);

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
    if(nUpdate == 0) {
        pMadgwick->getRotation(&q0, &q1, &q2, &q3);
        pGLWidget->setRotation(q0, q1, q2, q3);
        pGLWidget->update();

        ui->XgyroEdit->setText(QString("%1").arg(values[3], 4, 'f', 0));
        ui->YgyroEdit->setText(QString("%1").arg(values[4], 4, 'f', 0));
        ui->ZgyroEdit->setText(QString("%1").arg(values[5], 4, 'f', 0));

        ui->XaccEdit->setText(QString("%1").arg(values[0], 4, 'f', 0));
        ui->YaccEdit->setText(QString("%1").arg(values[1], 4, 'f', 0));
        ui->ZaccEdit->setText(QString("%1").arg(values[2], 4, 'f', 0));

        ui->XmagEdit->setText(QString("%1").arg(values[6], 4, 'f', 0));
        ui->YmagEdit->setText(QString("%1").arg(values[7], 4, 'f', 0));
        ui->ZmagEdit->setText(QString("%1").arg(values[8], 4, 'f', 0));

//        ui->headingEdit->setText(QString("%1").arg(1.0f/delta));
//        ui->psiEdit->setText(QString("%1").arg(pMadgwick->getRoll(), 0, 'f', 1));
//        ui->thetaEdit->setText(QString("%1").arg(pMadgwick->getPitch(), 0, 'f', 1));
//        ui->phiEdit->setText(QString("%1").arg(pMadgwick->getYaw(), 0, 'f', 1));
    }
}
