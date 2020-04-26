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
{
    ui->setupUi(this);

    pAcc  = new ADXL345(); // init ADXL345
    pAcc->init(FIMU_ACC_ADDR);

    pGyro = new ITG3200(); // init ITG3200
    pGyro->init(FIMU_ITG3200_DEF_ADDR);
    QThread::msleep(1000);
    pGyro->zeroCalibrate(128, 5); // calibrate the ITG3200


    pMagn = new HMC5883L();// init HMC5883L
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

    sampleFrequency = 200;
    pMadgwick->begin(sampleFrequency);

    connect(&loopTimer, SIGNAL(timeout()),
            this, SLOT(onLoopTimeElapsed()));

    lastUpdate = micros();
    now = 0;
    loopTimer.setTimerType(Qt::PreciseTimer);
    loopTimer.start(int32_t(1000.0/sampleFrequency+0.5));
}


MainWindow::~MainWindow() {
    delete ui;
}


void
MainWindow::onLoopTimeElapsed() {
    now = micros();
    pMadgwick->setInvFreq(float(now-lastUpdate)/1000000.f);
    lastUpdate = now;

    pAcc->get_Gxyz(&values[0]);
    pGyro->readGyro(&values[3]);
    pMagn->ReadScaledAxis(&values[6]);

    pMadgwick->update(values[3], values[4], values[5],
                      values[0], values[1], values[2],
                      values[6], values[7], values[8]);

//    pMadgwick->updateIMU(values[3], values[4], values[5],
//                         values[0], values[1], values[2]);

    nUpdate++;
    nUpdate = nUpdate % 100;
    if(nUpdate == 0) {
        ui->psiEdit->setText(QString("%1").arg(pMadgwick->getRoll(), 0, 'f', 1));
        ui->thetaEdit->setText(QString("%1").arg(pMadgwick->getPitch(), 0, 'f', 1));
        ui->phiEdit->setText(QString("%1").arg(pMadgwick->getYaw(), 0, 'f', 1));
    }
}
