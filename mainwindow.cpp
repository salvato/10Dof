#include "mainwindow.h"
#include "ui_mainwindow.h"


// Addresses :
// 0x1E     HMC5883L_Address        (Magnetometer)
// 0x53     ADXL345_ADDR_ALT_LOW    (Accelerometer)
// 0x68     ITG3200_ADDR_AD0_LOW    (Gyroscope)
// 0x77     BMP085_ADDRESS          (Pressure and Temperature)


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , pSixDOF(nullptr)
    , pCompass(nullptr)
{
    ui->setupUi(this);
    pSixDOF  = new FreeSixIMU();
    pCompass = new HMC5883L();
}


MainWindow::~MainWindow()
{
    delete ui;
}
