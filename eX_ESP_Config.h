#ifndef EX_ESP_CONFIG_H
#define EX_ESP_CONFIG_H

#define  USE_UART

#ifndef USE_UART
 #define USE_2_SERVOS
#endif

/// I2C and UART bus parameters ///
#define  I2C_SPEED                  400000
#define  SERIAL_SPEED               115200
//#define  K_bat                      0.004581469370719
//#define  K_bat                      1.5*0.004581469370719 //rz: alternative battery level divider
#define  K_bat                      1.0 / 1024 * 3.3 / (2.2/(2.2+22)) * 0.98 // last is error factor

///    Parameters of the MPU6050 module      ///
//============= start ================//
#define  ACCEL_SCALE_G              8192             // (2G range) G = 8192
#define  ACCEL_WEIGHT               0.01
#define  GYRO_BIAS_WEIGHT           0.005
// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define  Gyro_Gain                  0.03048
#define  Gyro_Scaled(x)             x*Gyro_Gain      //Return the scaled gyro raw data in degrees per second
#define  RAD2GRAD                   57.2957795
#define  GRAD2RAD                   0.01745329251994329576923690768489
//============== end =================//

// Set the parameters of the driver and driver type //
//============= start ================//
#define  MICROSTEPPING              4    // 8, 16, 32
#define  ANGLE_PER_STEP             1.8   // 0.9, 1.8
#define  K_MOTOR_SPEED              MICROSTEPPING * 360 / ANGLE_PER_STEP
#define  PRE_DIR_STROBE             25    // 25 - 640 ns, 27 - 690ns, 28 - 740 ns
//============== end =================//

///       Constructive section      ///
//============= start ================//
// For the original robot, a wheel with D = 100 mm was used
// For this wheel, the distance for one turn (Périmeter) will be:  Pi*D = 314.15652 mm
// The number of steps for the used motor is  200 steps and 1.8 degrés equivalent to 1600 microsteps at 1/8 
// for a setup with microsteps at 1/16, the number of microsteps will double and become 3200 steps.

// The controlling value for maximum speed is 500 
// This number multiplied by 23 gives the motor speed
// This allows us to calculate the period of one cycle equivalent to 173.913 us. As the puls is 1/2 of the period
// this results in a pulse of 86.9565 us.
// rz rem: my calculation of 1/period = 1/11500 = 86.9565us and a plus if 43us...... to be checked !!!!!!!!!!!!!!!!
// which corresponds to a frequency of 11.5 kHz (500*23=11500) 
// with this frequency, the motor makes 7.1875 turns per second 
// which corrresponds to a linear speed of 2258 mm/s = 8.1288 km/h
 
#define  MAX_SPEED_LINEAR           8.5   // km / h
//#define  VHEEL_DIAMETR              108   // mm
#define VHEEL_DIAMETR               94 //rz: mm

#define  MAX_TURN                   MAX_SPEED_LINEAR * 1000000 / 3600 / VHEEL_DIAMETR / PI  // revolutions per second
//#define  MAX_FREQUENCY              MAX_TURN * 360 / ANGLE_PER_STEP * MICROSTEPPING         // Maximum motor STEP frequency (Hz)

//============== end =================//


///     Manageability parameters      ///
//============= start ================//
#define  ZERO_SPEED                 25000
#define  MAX_ACCEL                  8
#define  MAX_CONTROL_OUTPUT         500

// NORMAL MODE = smooth, moderately
#define  MAX_THROTTLE               480      // speed
#define  MAX_STEERING               130      // max steering
#define  MAX_TARGET_ANGLE           12       // max tilt angle

// PRO MODE = more aggressive
#define  MAX_THROTTLE_PRO           680
#define  MAX_STEERING_PRO           250
#define  MAX_TARGET_ANGLE_PRO       20
//============== end =================//

///         PID-options            ///
//============= start ================//
// Default management conditions
#define  KP                         0.07    // alternative values: 0.20, 0.22 | 0.07
#define  KD                         8      // 26 28 | 12
#define  KP_THROTTLE                0.10    // 0.065, 0.08 | 0.07
#define  KI_THROTTLE                0.04    // 0.05 | 0.04
// Increase in control when the robot is raised from a recumbent position
#define  KP_RAISEUP                 0.10
#define  KD_RAISEUP                 0.05
#define  KP_THROTTLE_RAISEUP        0       // When lifting, the speed of the motors is not controlled
#define  KI_THROTTLE_RAISEUP        0.0
#define  ITERM_MAX_ERROR            40      // Iterm windup constants
#define  ITERM_MAX                  5000
//============== end =================//

/// Servo Drive Parameters ///
//============= start ================//
// the maximum value of the period is 20 milliseconds and with a cycle time of 10 μc = 2000 cycles
// the positive strobe of the control pulse changes within:
// the minimum is 0.6 ms, the average is 1.5 ms and the largest is 2.4 ms or in the periods 60-150-240
//
#define  SERVO_AUX_NEUTRO           250     // neutral servo position, was 150
#define  SERVO_MIN_PULSEWIDTH       100
#define  SERVO_MAX_PULSEWIDTH       400
//============== end =================//
#endif // EX_ESP_CONFIG_H
