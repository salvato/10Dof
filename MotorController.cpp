#include "MotorController.h"
#include "pigpiod_if2.h" // The library to use GPIO pins on Raspberry


#include <algorithm> // min() & max()
#include "string.h"  // for memset()
#include <QDebug>
#include <QThread>

using namespace std; // min() & max()


#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1


MotorController::MotorController(uint32_t _ena, uint32_t _in1, uint32_t _in2,
                                 uint32_t _enb, uint32_t _in3, uint32_t _in4,
                                 double _motor1Const, double _motor2Const)
{
    pwm1Pin    = _ena;
    mot1in1Pin = _in1;
    mot1in2Pin = _in2;

    pwm2Pin    = _enb;
    mot2in1Pin = _in3;
    mot2in2Pin = _in4;

    motor1Const = _motor1Const;
    motor2Const = _motor2Const;

    //===========================================================================
    // Each GPIO can be independently set to one of 18 different PWM frequencies.
    // The selectable frequencies depend upon the sample rate which may be
    // 1, 2, 4, 5, 8, or 10 microseconds (default 5).
    // The sample rate is set when the pigpio daemon is started (Default 5).
    //    The frequencies for each sample rate are:
    //===========================================================================
    //                                 Hertz
    //---------------------------------------------------------------------------
    //           1: 40000 20000 10000 8000 5000 4000 2500 2000 1600
    //               1250  1000   800  500  400  250  200  100   50
    //
    //           2: 20000 10000  5000 4000 2500 2000 1250 1000  800
    //                625   500   400  250  200  125  100   50   25
    //
    //           4: 10000  5000  2500 2000 1250 1000  625  500  400
    //                313   250   200  125  100   63   50   25   13
    //    sample
    //     rate
    //     (us)  5:  8000  4000  2000 1600 1000  800  500  400  320
    //                250   200   160  100   80   50   40   20   10
    //
    //           8:  5000  2500  1250 1000  625  500  313  250  200
    //                156   125   100   63   50   31   25   13    6
    //
    //          10:  4000  2000  1000  800  500  400  250  200  160
    //                125   100    80   50   40   25   20   10    5
    //===========================================================================

    PWMfrequency    = 5000; // in Hz

    gpioHostHandle = pigpio_start((char*)"localhost", (char*)"8888");
    if(gpioHostHandle < 0) {
        qDebug() << QString("Unable to initialize GPIO");
        exit(EXIT_FAILURE);
    }

    // set_PWM_frequency() returns the numerically closest frequency if OK
    int32_t iRealPWMfreq = set_PWM_frequency(gpioHostHandle, pwm1Pin, PWMfrequency);
    if(iRealPWMfreq < 0) {
        qDebug() << QString("Non riesco a definire la frequenza del PWM per il Pan.");
        exit(EXIT_FAILURE);
    }
    PWMfrequency = iRealPWMfreq; // Now it is the Real PWM frequency
    if(set_servo_pulsewidth(gpioHostHandle, pwm1Pin, 0) < 0) {
        qDebug() << QString("Non riesco a far partire il PWM per il Motore 1.");
        exit(EXIT_FAILURE);
    }
    set_PWM_frequency(gpioHostHandle, pwm1Pin, 0); // To avoid oscillations

    if(set_PWM_frequency(gpioHostHandle, pwm2Pin, PWMfrequency) < 0) {
        qDebug() << QString("Non riesco a definire la frequenza del PWM per il Motore 2.");
        exit(EXIT_FAILURE);
    }
    if(set_servo_pulsewidth(gpioHostHandle, pwm2Pin, 0) < 0) {
        qDebug() << QString("Non riesco a far partire il PWM per il Motore 2.");
        exit(EXIT_FAILURE);
    }
    set_PWM_frequency(gpioHostHandle, pwm2Pin, 0); // To avoid oscillations
    //=======================================================================
    // The real range, the number of steps between fully off and fully
    // on for each of the 18 available GPIO frequencies is
    //=======================================================================
    //      25(#1),     50(#2),   100(#3),   125(#4),    200(#5),    250(#6),
    //     400(#7),    500(#8),   625(#9),  800(#10),  1000(#11),  1250(#12),
    //    2000(#13), 2500(#14), 4000(#15), 5000(#16), 10000(#17), 20000(#18).
    //=======================================================================
    // The real value set by set_PWM_range() is (dutycycle*real_range)/range.
    //=======================================================================

    initPins();
}


void
MotorController::initPins() {

    // Motor 1 Pins
    if(set_mode(gpioHostHandle, mot1in1Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot1in1Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot1in1Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up") .arg(mot1in1Pin);
        exit(EXIT_FAILURE);
    }

    if(set_mode(gpioHostHandle, mot1in2Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot1in2Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot1in2Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up").arg(mot1in2Pin);
        exit(EXIT_FAILURE);
    }
    // Motor 2 Pins
    if(set_mode(gpioHostHandle, mot2in1Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot2in1Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot2in1Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up").arg(mot2in1Pin);
        exit(EXIT_FAILURE);
    }

    if(set_mode(gpioHostHandle, mot2in2Pin, PI_OUTPUT) < 0) {
        qDebug() << QString("Unable to initialize GPIO%1 as Output").arg(mot2in2Pin);
        exit(EXIT_FAILURE);
    }
    else if(set_pull_up_down(gpioHostHandle, mot2in2Pin, PI_PUD_UP) < 0) {
        qDebug() << QString("Unable to set GPIO%1 Pull-Up").arg(mot2in2Pin);
        exit(EXIT_FAILURE);
    }
}


MotorController::~MotorController() {
    if(gpioHostHandle>=0) {
        pigpio_stop(gpioHostHandle);
    }
}


int32_t
MotorController::map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void
MotorController::move(int leftSpeed, int rightSpeed, int minAbsSpeed) {
    if (rightSpeed < 0) {
        rightSpeed = min(rightSpeed, -1*minAbsSpeed);
        rightSpeed = max(rightSpeed, -255);
    }
    else if (rightSpeed > 0) {
        rightSpeed = max(rightSpeed, minAbsSpeed);
        rightSpeed = min(rightSpeed, 255);
    }
    
    int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

    if (leftSpeed < 0) {
        leftSpeed = min(leftSpeed, -1*minAbsSpeed);
        leftSpeed = max(leftSpeed, -255);
    }
    else if (leftSpeed > 0) {
        leftSpeed = max(leftSpeed, minAbsSpeed);
        leftSpeed = min(leftSpeed, 255);
    }
    
    int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);

    gpio_write(gpioHostHandle, mot2in1Pin, rightSpeed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, rightSpeed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot1in1Pin, leftSpeed  > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, leftSpeed  > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
    TIM3->CCR1 = realRightSpeed * motor1Const; // ???????
    //    sConfig.Pulse = realRightSpeed * motor1Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    // Set the pulse value for channel 2
    TIM3->CCR2 = realLeftSpeed  * motor2Const; // ???????
    //    sConfig.Pulse = realLeftSpeed  * motor2Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 1
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 2
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
}


void
MotorController::move(int speed, int minAbsSpeed) {
    int direction = 1;
    
    if (speed < 0) {
        direction = -1;
        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }
    
    if (speed == currentSpeed) return;
    
    int realSpeed = max(minAbsSpeed, abs(speed));
    
    gpio_write(gpioHostHandle, mot1in1Pin, speed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot2in1Pin, speed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
    TIM3->CCR1 = realSpeed * motor1Const; // ???????
    //    sConfig.Pulse = realSpeed * motor1Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    // Set the pulse value for channel 2
    TIM3->CCR2 = realSpeed * motor2Const; // ???????
    //    sConfig.Pulse = realSpeed  * motor2Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 1
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 2
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }

    currentSpeed = direction * realSpeed;
}


void
MotorController::move(int speed) {
    if (speed == currentSpeed) return;
    
    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;
    
    gpio_write(gpioHostHandle, mot1in1Pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot2in1Pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
    TIM3->CCR1 = abs(speed) * motor1Const; // ???????
    //    sConfig.Pulse = abs(speed) * motor1Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    // Set the pulse value for channel 2
    TIM3->CCR2 = abs(speed)  * motor2Const; // ???????
    //    sConfig.Pulse = abs(speed)  * motor2Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 1
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 2
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }

    currentSpeed = speed;
}


void
MotorController::turnLeft(int speed, bool kick) {
    gpio_write(gpioHostHandle, mot1in1Pin, GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot1in2Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, GPIO_PIN_SET);
    
    if (kick) {
        TIM3->CCR1 = 255; // ???????
        TIM3->CCR2 = 255; // ???????
        //        sConfig.Pulse = 255;
        //        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
        //            // Configuration Error
        //            Error_Handler();
        //        }
        //        // Set the pulse value for channel 2
        //        sConfig.Pulse = 255;
        //        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
        //            // Configuration Error
        //            Error_Handler();
        //        }
        //        // Start PWM signal on channel 1
        //        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
        //            // PWM Generation Error
        //            Error_Handler();
        //        }
        //        // Start PWM signal on channel 2
        //        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
        //            // PWM Generation Error
        //            Error_Handler();
        //        }
        QThread::msleep(100);
    }
    
    // Set the pulse value for channel 1
    TIM3->CCR1 = speed * motor1Const; // ???????
    TIM3->CCR2 = speed * motor2Const; // ???????
    //    sConfig.Pulse = speed * motor1Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Set the pulse value for channel 2
    //    sConfig.Pulse = speed * motor2Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 1
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 2
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
}


void
MotorController::turnRight(int speed, bool kick) {
    gpio_write(gpioHostHandle, mot1in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, GPIO_PIN_SET);

    gpio_write(gpioHostHandle, mot2in1Pin, GPIO_PIN_SET);
    gpio_write(gpioHostHandle, mot2in2Pin, GPIO_PIN_RESET);

    if (kick) {
        TIM3->CCR1 = 255; // ???????
        TIM3->CCR2 = 255; // ???????
        // Set the pulse value for channel 1
        //        sConfig.Pulse = 255;
        //        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
        //            // Configuration Error
        //            Error_Handler();
        //        }
        //        // Set the pulse value for channel 2
        //        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
        //            // Configuration Error
        //            Error_Handler();
        //        }
        //        // Start PWM signal on channel 1
        //        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
        //            // PWM Generation Error
        //            Error_Handler();
        //        }
        //        // Start PWM signal on channel 2
        //        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
        //            // PWM Generation Error
        //            Error_Handler();
        //        }
        QThread::msleep(100);
    }
    // Set the pulse value for channel 1
    TIM3->CCR1 = speed * motor1Const; // ???????
    TIM3->CCR2 = speed * motor2Const; // ???????
    //    sConfig.Pulse = speed * motor1Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Set the pulse value for channel 2
    //    sConfig.Pulse = speed * motor2Const;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 1
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 2
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
}


void
MotorController::stopMoving() {
    gpio_write(gpioHostHandle, mot1in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot1in2Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in1Pin, GPIO_PIN_RESET);
    gpio_write(gpioHostHandle, mot2in2Pin, GPIO_PIN_RESET);

    TIM3->CCR1 = 0; // ???????
    TIM3->CCR2 = 0; // ???????
    //    // Set the pulse value for channel 1
    //    sConfig.Pulse = 0;
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Set the pulse value for channel 2
    //    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    //        // Configuration Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 1
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }
    //    // Start PWM signal on channel 2
    //    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
    //        // PWM Generation Error
    //        Error_Handler();
    //    }

    currentSpeed = 0;
}
