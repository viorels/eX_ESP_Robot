#ifndef EX_ESP_PINS_H
#define EX_ESP_PINS_H

// GPIO0    : v starting conditions - 10K to + 3.3V, or short-circuit to ground for reflashing             
// GPIO1    : x U0TXD (Serial1)
// GPIO2    : v starting conditions - 10K to + 3.3V
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
// GPIO15   : v starting conditions - 10K on the ground
// GPIO16   : v
// T_OUT    : v Analog pin (17)

#define MOTORS_ENABLE_PIN    D8 // A 10k resistor is not nécessary as integrated inside the motor driver
#define MOTORS_DIR_PIN       D7
#define MOTOR1_STEP_PIN      D5
#define MOTOR2_STEP_PIN      D6

#define SONAR_TRIG_PIN       16 // D0 !!! this and no other !!!
#define SONAR_ECHO_PIN       0  // D3
#define SDA_PIN              4  // D2
#define SCL_PIN              5  // D1

#define SERVO1_PIN           2  // D4
#define SERVO2_PIN           -1

#define MICROSTEPPING_PIN           3  // RX
#define BATTERY_PIN          17 // A0?

#define LED_RED              16 //rz: This to check gyro trouble as gyro gets in trouble during code upload

#endif //EX_ESP_PINS_H
