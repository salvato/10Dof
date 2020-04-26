#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

#include <cmath>


// Addresses :
// 0x1E     HMC5883L_Address        (Magnetometer)
// 0x53     ADXL345_ADDR_ALT_LOW    (Accelerometer)
// 0x68     ITG3200_ADDR_AD0_LOW    (Gyroscope)
// 0x77     BMP085_ADDRESS          (Pressure and Temperature)


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , pSixDOF(nullptr)
{
    ui->setupUi(this);

    pSixDOF  = new FreeSixIMU();
    pSixDOF->init(); // init the Acc, Gyro and Magn

    // delay(5);

    //bmp085Calibration(); // init barometric pressure sensor

    connect(&loopTimer, SIGNAL(timeout()),
            this, SLOT(onLoopTimeElapsed()));

    loopTimer.start(2);
}


MainWindow::~MainWindow() {
    delete ui;
}


void
MainWindow::onLoopTimeElapsed() {
    pSixDOF->getEuler(angles);
    ui->psiEdit->setText(QString("%1").arg(angles[0], 0, 'f', 1));
    ui->thetaEdit->setText(QString("%1").arg(angles[1], 0, 'f', 1));
    ui->phiEdit->setText(QString("%1").arg(angles[2], 0, 'f', 1));

//    temperature = bmp085GetTemperature(bmp085ReadUT());
//    pressure = bmp085GetPressure(bmp085ReadUP());
/*
    getHeading();
    ui->headingEdit->setText(QString("%1").arg(heading));

    bool showRaw = false;

    if(showRaw) {
        int16_t raw_values[9];
        pSixDOF->getRawValues(raw_values);

        ui->XaccEdit->setText(QString("%1").arg(raw_values[0]));
        ui->YaccEdit->setText(QString("%1").arg(raw_values[1]));
        ui->ZaccEdit->setText(QString("%1").arg(raw_values[2]));

        ui->XgyroEdit->setText(QString("%1").arg(raw_values[3]));
        ui->YgyroEdit->setText(QString("%1").arg(raw_values[4]));
        ui->ZgyroEdit->setText(QString("%1").arg(raw_values[5]));

        ui->XmagEdit->setText(QString("%1").arg(raw_values[6]));
        ui->YmagEdit->setText(QString("%1").arg(raw_values[7]));
        ui->ZmagEdit->setText(QString("%1").arg(raw_values[8]));
    }
    else {
        float   values[9];
        pSixDOF->getValues(values);

        ui->XaccEdit->setText(QString("%1").arg(values[0], 0, 'f', 1));
        ui->YaccEdit->setText(QString("%1").arg(values[1], 0, 'f', 1));
        ui->ZaccEdit->setText(QString("%1").arg(values[2], 0, 'f', 1));

        ui->XgyroEdit->setText(QString("%1").arg(values[3], 0, 'f', 1));
        ui->YgyroEdit->setText(QString("%1").arg(values[4], 0, 'f', 1));
        ui->ZgyroEdit->setText(QString("%1").arg(values[5], 0, 'f', 1));

        ui->XmagEdit->setText(QString("%1").arg(values[6], 0, 'f', 1));
        ui->YmagEdit->setText(QString("%1").arg(values[7], 0, 'f', 1));
        ui->ZmagEdit->setText(QString("%1").arg(values[8], 0, 'f', 1));
    }

    //    PrintData();
*/
}

void
MainWindow::getHeading() {
    // Retrived the scaled values from the compass (scaled to the configured scale).
    float   values[9];
    pSixDOF->getValues(values);

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    heading = atan2(values[7], values[6]);

    float declinationAngle = 0.0457;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*M_PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*M_PI)
        heading -= 2*M_PI;

    // Convert radians to degrees for readability.
    heading = heading * 180/M_PI;
}


void
MainWindow::PrintData() {

//    qDebug() << "Pressure: " << pressure << " Pa";
}
