//#define L298
#define BST760


#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "plot2d.h"
#include <QtWidgets>
#include <QDebug>
#include <QThread>
#include <GLwidget.h>
#include <QPushButton>
#include <QSettings>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>


#include "PID_v1.h"
#if defined(L298)
    #include "MotorController_L298.h"
#elif defined(BST760)
    #include "MotorController_BST7960.h"
#else
    #error "Undefined Motor Controller"
#endif

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

#if defined(L298)
    // L298 MotorController
    #define PWM1_PIN  12 // on BCM12: Pin 32 in the 40 pin GPIO connector.
    #define M1IN1_PIN 17 // on BCM17: Pin 11 in the 40 pin GPIO connector.
    #define M1IN2_PIN 27 // on BCM27: Pin 13 in the 40 pin GPIO connector.
    #define PWM2_PIN  13 // on BCM13: Pin 33 in the 40 pin GPIO connector.
    #define M2IN1_PIN 22 // on BCM22: Pin 15 in the 40 pin GPIO connector.
    #define M2IN2_PIN 23 // on BCM23: Pin 16 in the 40 pin GPIO connector.
#elif defined(BST760)
    // BST760 MotorController
    #define PWM1UP_PIN  12 // on BCM12: Pin 32 in the 40 pin GPIO connector.
    #define PWM1LOW_PIN 13 // on BCM13: Pin 33 in the 40 pin GPIO connector.
    #define PWM2UP_PIN  22 // on BCM22: Pin 15 in the 40 pin GPIO connector.
    #define PWM2LOW_PIN 23 // on BCM23: Pin 16 in the 40 pin GPIO connector.
#else
    #error "Undefined Motor Controller"
#endif


#define MIN_ABS_SPEED 0


//==============================================================
// Information for connecting servos:
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


// The magnitude of the Earth's magnetic field at its surface
// ranges from 250 to 650 milli Gauss.


MainWindow::MainWindow()
    : pAcc(nullptr)
    , pGyro(nullptr)
    , pMagn(nullptr)
    , pMadgwick(nullptr)
    , pPid(nullptr)
    , pMotorController(nullptr)
    // Widgets
    , pGLWidget(nullptr)
    , pPlotVal(nullptr)
    // Status Variables
    , bRunInProgress(false)
    , bAccCalInProgress(false)
    , bGyroCalInProgress(false)
    , bMagCalInProgress(false)
    , bShowPidInProgress(false)
    , bShow3DInProgress(true)
    // TCP-IP Server
    , bNetworkAvailable(false)
    , pTcpServer(nullptr)
    , pTcpServerConnection(nullptr)
    , serverPort(43210)
    // Motor Controller
    , motorSpeedFactorLeft(0.6)
    , motorSpeedFactorRight(0.5)
{
    restoreSettings();
    if(openTcpSession()) bNetworkAvailable = true;

    initLayout();
    initAHRSsensor();
#if defined(L298)
    pMotorController = new MotorController_L298(PWM1_PIN, M1IN1_PIN, M1IN2_PIN,
                                                PWM2_PIN, M2IN1_PIN, M2IN2_PIN,
                                                motorSpeedFactorLeft, motorSpeedFactorRight);
#elif defined(BST760)
    pMotorController = new MotorController_BST7960(PWM1UP_PIN, PWM1LOW_PIN,
                                                   PWM2UP_PIN, PWM2LOW_PIN,
                                                   motorSpeedFactorLeft, motorSpeedFactorRight);
#endif
    pPid = new PID(Kp, Ki, Kd, ControllerDirection);
    setpoint = 0.0;
    pPid->SetMode(AUTOMATIC);
    pPid->SetSampleTime(100); // in ms
    pPid->SetOutputLimits(-255, 255);

    samplingFrequency = 300;
    pMadgwick = new Madgwick();
    pMadgwick->begin(samplingFrequency);

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

    for(int i=0; i<10000; i++) {
        pMadgwick->update(values[3], values[4], values[5],
                values[0], values[1], values[2],
                values[6], values[7], values[8]);
    }

    if(!bNetworkAvailable) {
        pMadgwick->getRotation(&q0, &q1, &q2, &q3);
        pGLWidget->setRotation(q0, q1, q2, q3);
    }
}


MainWindow::~MainWindow() {
}


void
MainWindow::closeEvent(QCloseEvent *event) {
    Q_UNUSED(event)
    pMotorController->stopMoving();
    loopTimer.stop();

    saveSettings();

    if(pTcpServer) {
        pTcpServer->close();
        delete pTcpServer;
    }
    if(pPid) delete pPid;
    if(pMadgwick) delete pMadgwick;
    if(pMotorController) delete pMotorController;
    if(pMagn) delete pMagn;
    if(pGyro) delete pGyro;
    if(pAcc) delete pAcc;
}


void
MainWindow::restoreSettings() {
    QSettings settings;

    // Restore Geometry and State of the window
    restoreGeometry(settings.value("Geometry").toByteArray());
    // Gyroscope
    GyroXOffset = settings.value("GyroXOffset", 1.0).toFloat();
    GyroYOffset = settings.value("GyroYOffset", 1.0).toFloat();
    GyroZOffset = settings.value("GyroZOffset", 1.0).toFloat();
    // PID
    ControllerDirection = settings.value("ControllerDirection", 0).toInt();
    Kp                  = settings.value("Kp",                  1.0).toDouble();
    Kd                  = settings.value("Kd",                  0.0).toDouble();
    Ki                  = settings.value("Ki",                  0.0).toDouble();
    // Motor Controller
    motorSpeedFactorLeft = settings.value("motorSpeedFactorLeft",  0.6).toDouble();
    motorSpeedFactorRight= settings.value("motorSpeedFactorRight", 0.5).toDouble();
}


void
MainWindow::saveSettings() {
    QSettings settings;

    // Window Position and Size
    settings.setValue("Geometry", saveGeometry());
    // Gyroscope
    settings.setValue("GyroXOffset", pGyro->offsets[0]);
    settings.setValue("GyroYOffset", pGyro->offsets[1]);
    settings.setValue("GyroZOffset", pGyro->offsets[2]);
    // PID
    settings.setValue("ControllerDirection", ControllerDirection);
    settings.setValue("Kp", Kp);
    settings.setValue("Kd", Kd);
    settings.setValue("Ki", Ki);
    // Motor Controller
    settings.setValue("motorSpeedFactorLeft",  motorSpeedFactorLeft);
    settings.setValue("motorSpeedFactorRight", motorSpeedFactorRight);
}


bool
MainWindow::openTcpSession() {
    pTcpServer = new QTcpServer(this);
    if(!pTcpServer->listen(QHostAddress::Any, serverPort)) {
        qDebug()  << "TCP-IP Unable to start listen()";
        return false;
    }
    connect(pTcpServer, SIGNAL(newConnection()),
            this, SLOT(onNewTcpConnection()));
    connect(pTcpServer, SIGNAL(acceptError(QAbstractSocket::SocketError)),
            this, SLOT(onTcpError(QAbstractSocket::SocketError)));
    QString ipAddress;
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
    // use the first non-localhost IPv4 address
    for(qint32 i=0; i<ipAddressesList.size(); ++i) {
        if(ipAddressesList.at(i) != QHostAddress::LocalHost && ipAddressesList.at(i).toIPv4Address()) {
            ipAddress = ipAddressesList.at(i).toString();
            if(ipAddress.left(3) != QString("169"))
                break;
        }
    }
    // if we did not find one, use IPv4 localhost
    if(ipAddress.isEmpty())
        ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
    qDebug() << QString("Running TCP-IP server at address %1 port:%2")
                .arg(ipAddress)
                .arg(pTcpServer->serverPort());
    return true;
}


void
MainWindow::onTcpError(QAbstractSocket::SocketError error) {
    if(error == QAbstractSocket::ConnectionRefusedError)
        qDebug() << "The connection was refused by the peer (or timed out).";
    else if(error == QAbstractSocket::RemoteHostClosedError) {
        qDebug() << " The remote host closed the connection.";
    } else if(error == QAbstractSocket::HostNotFoundError)
        qDebug() << " The host address was not found.";
    else if(error == QAbstractSocket::SocketAccessError)
        qDebug() << " The socket operation failed because the application lacked the required privileges.";
    else if(error == QAbstractSocket::SocketResourceError)
        qDebug() << " The local system ran out of resources (e.g., too many sockets).";
    else if(error == QAbstractSocket::SocketTimeoutError)
        qDebug() << " The socket operation timed out.";
    else if(error == QAbstractSocket::DatagramTooLargeError)
        qDebug() << " The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
    else if(error == QAbstractSocket::NetworkError)
        qDebug() << " An error occurred with the network (e.g., the network cable was accidentally plugged out).";
    else if(error == QAbstractSocket::AddressInUseError)
        qDebug() << " The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
    else if(error == QAbstractSocket::SocketAddressNotAvailableError)
        qDebug() << " The address specified to QAbstractSocket::bind() does not belong to the host.";
    else if(error == QAbstractSocket::UnsupportedSocketOperationError)
        qDebug() << " The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
    else if(error == QAbstractSocket::ProxyAuthenticationRequiredError)
        qDebug() << " The socket is using a proxy, and the proxy requires authentication.";
    else if(error == QAbstractSocket::SslHandshakeFailedError)
        qDebug() << " The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
    else if(error == QAbstractSocket::UnfinishedSocketOperationError)
        qDebug() << " Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
    else if(error == QAbstractSocket::ProxyConnectionRefusedError)
        qDebug() << " Could not contact the proxy server because the connection to that server was denied";
    else if(error == QAbstractSocket::ProxyConnectionClosedError)
        qDebug() << " The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
    else if(error == QAbstractSocket::ProxyConnectionTimeoutError)
        qDebug() << " The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
    else if(error == QAbstractSocket::ProxyNotFoundError)
        qDebug() << " The proxy address set with setProxy() (or the application proxy) was not found.";
    else if(error == QAbstractSocket::ProxyProtocolError)
        qDebug() << " The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
    else if(error == QAbstractSocket::OperationError)
        qDebug() << " An operation was attempted while the socket was in a state that did not permit it.";
    else if(error == QAbstractSocket::SslInternalError)
        qDebug() << " The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
    else if(error == QAbstractSocket::SslInvalidUserDataError)
        qDebug() << " Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
    else if(error == QAbstractSocket::TemporaryError)
        qDebug() << " A temporary error occurred (e.g., operation would block and socket is non-blocking).";
    else if(error == QAbstractSocket::UnknownSocketError)
        qDebug() << " An unidentified error occurred.";
}


void
MainWindow::onNewTcpConnection() {
    pTcpServerConnection = pTcpServer->nextPendingConnection();
    connect(pTcpServerConnection, SIGNAL(readyRead()),
            this, SLOT(onReadFromServer()));
    connect(pTcpServerConnection, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(onTcpError(QAbstractSocket::SocketError)));
    connect(pTcpServerConnection, SIGNAL(disconnected()),
            this, SLOT(onTcpClientDisconnected()));

    qDebug() << QString("Connected to: %1")
                .arg(pTcpServerConnection->peerAddress().toString());
}


void
MainWindow::onTcpClientDisconnected() {
    qDebug() << QString("Disconnection from: %1")
                .arg(pTcpServerConnection->peerAddress().toString());
    if(pTcpServerConnection) delete pTcpServerConnection;
    pTcpServerConnection = nullptr;
}


void
MainWindow::onReadFromServer() {
    clientMessage.append(pTcpServerConnection->readAll());
    QString sMessage = QString(clientMessage);
    int32_t iPos = sMessage.indexOf('#');
    while(iPos > 0) {
        QString sCommand = sMessage.left(iPos);
        executeCommand(sCommand);
        sMessage = sMessage.right(iPos+1);
        iPos = sMessage.indexOf('#');
    }
}


void
MainWindow::executeCommand(QString sMessage) {
    qDebug() << "Received: " << sMessage;
}


void
MainWindow::periodicUpdateWidgets() {
    if(pTcpServerConnection) {
        if(pTcpServerConnection->isOpen()) {
            QString message;
            message = QString("q %1 %2 %3 %4#")
                    .arg(q0)
                    .arg(q1)
                    .arg(q2)
                    .arg(q3);
            pTcpServerConnection->write(message.toLatin1());
        }
    }
}


void
MainWindow::createButtons() {
    buttonStartStop       = new QPushButton("Start",     this);
    buttonAccCalibration  = new QPushButton("Acc. Cal.", this);
    buttonGyroCalibration = new QPushButton("Gyro Cal.", this);
    buttonMagCalibration  = new QPushButton("Mag. Cal.", this);
    buttonShowPidOutput   = new QPushButton("Show PID",  this);
    buttonHide3D          = new QPushButton("Hide3D",    this);

    buttonStartStop->setEnabled(true);
    buttonHide3D->setEnabled(true);
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
    connect(buttonHide3D, SIGNAL(clicked()),
            this, SLOT(onHide3DPushed()));
}


void
MainWindow::createPlot() {
    pPlotVal = new Plot2D(this, "Plot");

    pPlotVal->NewDataSet(1, 1, QColor(255,   0,   0), Plot2D::ipoint, "X");
    pPlotVal->NewDataSet(2, 1, QColor(  0, 255,   0), Plot2D::ipoint, "Y");
    pPlotVal->NewDataSet(3, 1, QColor(  0,   0, 255), Plot2D::ipoint, "Z");
    pPlotVal->NewDataSet(4, 1, QColor(255, 255, 255), Plot2D::ipoint, "PID-In");
    pPlotVal->NewDataSet(5, 1, QColor(255, 255,  64), Plot2D::ipoint, "PID-Out");

    pPlotVal->SetShowTitle(1, true);
    pPlotVal->SetShowTitle(2, true);
    pPlotVal->SetShowTitle(3, true);
    pPlotVal->SetShowTitle(4, true);
    pPlotVal->SetShowTitle(5, true);

    pPlotVal->SetLimits(-1.0, 1.0, -1.0, 1.0, true, true, false, false);
}


void
MainWindow::initLayout() {
    if(!bNetworkAvailable) { // Local Display and Control
        createButtons();
        pGLWidget = new GLWidget(this);
        createPlot();
        QHBoxLayout *firstButtonRow = new QHBoxLayout;
        firstButtonRow->addWidget(buttonStartStop);
        firstButtonRow->addWidget(buttonHide3D);
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
}


bool
MainWindow::isStationary() {
    return false;
}


void
MainWindow::initAHRSsensor() {
    pAcc  = new ADXL345(); // init ADXL345
    pAcc->init(ACC_ADDR);
    pAcc->setRangeSetting(2); // +/-2g. Possible values are: 2g, 4g, 8g, 16g

    pGyro = new ITG3200(); // init ITG3200
    pGyro->init(ITG3200_DEF_ADDR);
    if(isStationary()) { // Gyro calibration done only when stationary
        QThread::msleep(1000);
        pGyro->zeroCalibrate(600, 10); // calibrate the ITG3200
    }
    else {
        pGyro->offsets[0] = GyroXOffset;
        pGyro->offsets[1] = GyroYOffset;
        pGyro->offsets[2] = GyroZOffset;
    }

    pMagn = new HMC5883L();// init HMC5883L
    pMagn->SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    pMagn->SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

    //bmp085Calibration(); // init barometric pressure sensor
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
        pMotorController->stopMoving();
        loopTimer.stop();
        bRunInProgress = false;
        if(bAccCalInProgress)
            onStartAccCalibration();
        if(bGyroCalInProgress)
            onStartGyroCalibration();
        if(bMagCalInProgress)
            onStartMagCalibration();
        if(bShowPidInProgress)
            onShowPidOutput();
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
        now = lastUpdate;
        update3D = 0;
        updatePlot = 0;
        loopTimer.start(int32_t(1000.0/samplingFrequency+0.5));
    }
}


void
MainWindow::onHide3DPushed() {
    if(bShow3DInProgress) {
        buttonHide3D->setText("Show 3D");
    }
    else {
        buttonHide3D->setText("Hide3D");
    }
    bShow3DInProgress = !bShow3DInProgress;
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
        pPlotVal->SetShowDataSet(5, false);

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
        pPlotVal->SetShowDataSet(5, false);

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
        pPlotVal->SetShowDataSet(5, false);

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
        pPlotVal->ClearDataSet(5);

        pPlotVal->SetShowDataSet(1, false);
        pPlotVal->SetShowDataSet(2, false);
        pPlotVal->SetShowDataSet(3, false);
        pPlotVal->SetShowDataSet(4, true);
        pPlotVal->SetShowDataSet(5, true);

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
        if(bAccCalInProgress && !bNetworkAvailable) {
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
        if(bGyroCalInProgress && !bNetworkAvailable) {
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
        if(bMagCalInProgress && !bNetworkAvailable) {
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

    input = pMadgwick->getPitch();
    output = pPid->Compute(input, setpoint);
    pMotorController->move(output, MIN_ABS_SPEED);

    update3D++;
    update3D %= 30;
    if(!update3D && bShow3DInProgress) {
        pMadgwick->getRotation(&q0, &q1, &q2, &q3);
        if(!bNetworkAvailable) {
            pGLWidget->setRotation(q0, q1, q2, q3);
            pGLWidget->update();
        }
        else {
            periodicUpdateWidgets();
        }
    }
    updatePlot++;
    updatePlot %= 150;
    if(!updatePlot && bShow3DInProgress && !bNetworkAvailable) {
        if(bShowPidInProgress | bAccCalInProgress  |
           bGyroCalInProgress | bMagCalInProgress  |
           bShowPidInProgress)
        {
            pPlotVal->UpdatePlot();
        }
    }
    if(bShowPidInProgress && bShow3DInProgress && !bNetworkAvailable) {
        double x = double(now-t0)/1000000.0;
        pPlotVal->NewPoint(4, x, double(input));
        pPlotVal->NewPoint(5, x, double(output/Kp));
//        pPlotVal->NewPoint(4, x, double(input-(output/Kp)));
//        pPlotVal->NewPoint(5, x, double(output/Kp));
    }
}
