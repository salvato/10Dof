#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>


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
    void getHeading();
    void PrintData();

private:
    Ui::MainWindow *ui;
    QTimer loopTimer;

    float angles[3]; // yaw pitch roll
    float heading;

    int16_t temperature;
    long pressure;

    // Set the FreeSixIMU object
    FreeSixIMU* pSixDOF;
//    HMC5883L* pCompass;

    // Record any errors that may occur in the compass.
    int error;
};

#endif // MAINWINDOW_H
