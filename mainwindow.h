#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <GLwidget.h>
#include <QTimer>

#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>


#include <sys/time.h>


#define FIMU_ACC_ADDR ADXL345_ADDR_ALT_LOW          // SDO connected to GND
//#define FIMU_BMA180_DEF_ADDR BMA180_ADDRESS_SDO_LOW
#define FIMU_ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW  // AD0 connected to GND
// HMC5843 address is fixed so don't bother to define it


namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void onLoopTimeElapsed();

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow *ui;
    GLWidget* pGLWidget;

    QTimer loopTimer;

    float sampleFrequency;
    float values[9];
    float angles[3]; // yaw pitch roll
    float heading;

    int16_t temperature;
    long pressure;

    ADXL345*  pAcc;
    ITG3200*  pGyro;
    HMC5883L* pMagn;
    Madgwick* pMadgwick;

    // Record any errors that may occur in the compass.
    int error;
    int nUpdate;
    uint64_t micros() {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
    }
    uint64_t lastUpdate;
    uint64_t now;
    float delta;
    float q0, q1, q2, q3;
};

#endif // MAINWINDOW_H
