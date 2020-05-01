#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QTimer>

#include <ADXL345.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>


#include <sys/time.h>


QT_FORWARD_DECLARE_CLASS(GLWidget)
QT_FORWARD_DECLARE_CLASS(QPushButton)
QT_FORWARD_DECLARE_CLASS(Plot2D)


#define FIMU_ACC_ADDR ADXL345_ADDR_ALT_LOW          // SDO connected to GND
//#define FIMU_BMA180_DEF_ADDR BMA180_ADDRESS_SDO_LOW
#define FIMU_ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW  // AD0 connected to GND
// HMC5843 address is fixed so don't bother to define it


class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

public slots:
    void onLoopTimeElapsed();
    void onStartAccCalibration();
    void onStartGyroCalibration();
    void onStartMagCalibration();

protected:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void initLayout();

private:
    GLWidget* pGLWidget;
    Plot2D*   pPlotVal;
    QPushButton* buttonAccCalibration;
    QPushButton* buttonGyroCalibration;
    QPushButton* buttonMagCalibration;

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
    uint64_t t0;
    float delta;
    float q0, q1, q2, q3;
    double angleX, angleY, angleZ;
    double avgX, avgY, avgZ;
    uint16_t nAvg, nCurr;
    bool bAccCalInProgress;
    bool bGyroCalInProgress;
    bool bMagCalInProgress;
};

#endif // MAINWINDOW_H
