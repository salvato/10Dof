#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
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

private:
    Ui::MainWindow *ui;

    float angles[3]; // yaw pitch roll
    float heading;

    short temperature;
    long pressure;

    // Set the FreeSixIMU object
    FreeSixIMU* pSixDOF;
    HMC5883L* pCompass;

    // Record any errors that may occur in the compass.
    int error;
};

#endif // MAINWINDOW_H
