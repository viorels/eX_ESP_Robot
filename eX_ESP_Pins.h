#ifndef EX_ESP_PINS_H
#define EX_ESP_PINS_H

// GPIO0    : v стартовые условия - 10К на +3.3В, или закоротить на землю для перепрошивки             
// GPIO1    : x U0TXD (Serial1)
// GPIO2    : v стартовые условия - 10К на +3.3В
// GPIO3    : x U0RXD (Serial1)
// GPIO4    : v
// GPIO5    : v
// GPIO6    : x SD_CLK
// GPIO7    : x SD_DATA0
// GPIO8    : x SD_DATA1
// GPIO9    : x SD_DATA2
// GPIO10   : x SD_DATA3
// GPIO11   : x SD_CMD
// GPIO12   : v
// GPIO13   : v
// GPIO14   : v
// GPIO15   : v стартовые условия - 10К на землю
// GPIO16   : v
// T_OUT    : v Analog pin (17)

#ifdef ORIGINAL_PINS
#define MOTORS_ENABLE_PIN    15 // резистор 10К на землю может не устанавливаться, так как резисторы интегрированы в драйвера ШД
#define MOTORS_DIR_PIN       14
#define MOTOR1_STEP_PIN      12
#define MOTOR2_STEP_PIN      13

#define SONAR_TRIG_PIN       16 // !!! этот и никакой другой !!!
#define SONAR_ECHO_PIN       0
#define SDA_PIN              4 // или 5, для ESP-07 (плата разведена не правильно)
#define SCL_PIN              5 // или 4, для ESP-07 (плата разведена не правильно)

#define SERVO1_PIN           2
#define SERVO2_PIN           3
#define BATTERY_PIN          17

#else // rolfz pinout
#define MOTORS_ENABLE_PIN    12 //D6 //rz: A 10k resistor is not nécessary as integrated inside the motor driver
#define MOTORS_DIR_PIN       15 //D8  //14
#define MOTOR1_STEP_PIN      13 //D7
#define MOTOR2_STEP_PIN      14 //D5

#define SONAR_TRIG_PIN       -1 //16 // NO sonar in my first check
#define SONAR_ECHO_PIN       -1 //0
#define SDA_PIN              4 // ou 5 pour ESP-07 (плата разведена не правильно)
#define SCL_PIN              5 // ou 4 pour ESP-07 (плата разведена не правильно)

#define SERVO1_PIN           2
#define SERVO2_PIN           3
#define BATTERY_PIN          17
 
#define LED_RED              16 //rz: This to check gyro trouble as gyro gets in trouble during code upload
#endif

#endif //EX_ESP_PINS_H
