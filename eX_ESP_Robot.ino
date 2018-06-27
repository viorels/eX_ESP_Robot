// eX-Robot  Балансирующий робот с шаговым приводом
// Author: Oleg Kravtchenko
// Date: 26/10/2015
// License: GPL v2

// Some translations for my own understanding, by rolfz December 2016

// Для создания балансирующего робота использовано: 
// Material for the creation of the balancing robot:
// - 1 WiFi модуль ESP8266
// - 1 DMP processor MPU6050
// - 2 Drivers Stepper Motors A4988
// - 2 Stepper Motors 17HS2408 (0.6A, 1.8 deg/step)
// Опционно:
// Options
// - 1 Futaba Micro 9g серво-привод
// - 1 Distance sensor (sonar) HC-SR04
// - 1 MP3 звуковой модуль
// - n RGB светодиод

#include <ESP8266WiFi.h>
#include "eX_ESP_Config.h"
#include "eX_ESP_Pins.h"
#include "eX_ESP_WiFi.h"
#include "eX_ESP_OSC.h"
#include "eX_ESP_MPU6050.h"
#include "eX_ESP_Functions.h"
#include "eX_ESP_Timing_Engine.h"

//#include <Arduino.h>
bool       robot_shutdown = false;   // Robot shutdown flag
bool       robot_pro_mode;           // Robot_mode = false - Normal mode, True - Pro mode ()

long       timer_old_value;
long       timer_value;
float      dt;
    
float      bat_level;
float      dist_value;

float      throttle;
float      steering;

float      max_throttle     = MAX_THROTTLE;
float      max_steering     = MAX_STEERING;
float      max_target_angle = MAX_TARGET_ANGLE;
float      Kp               = KP;
float      Kd               = KD;
float      Kp_thr           = KP_THROTTLE;
float      Ki_thr           = KI_THROTTLE;
float      Kp_user          = KP;
float      Kd_user          = KD;
float      Kp_thr_user      = KP_THROTTLE;
float      Ki_thr_user      = KI_THROTTLE;

float      angle_adjusted;            // Angle of the robot (used for stability control)
float      angle_adjusted_Old;
int16_t    actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t    actual_robot_speed_Old;    
float      estimated_speed_filtered;  // Estimated robot speed
float      target_angle;
float      control_output;

int16_t    motor1;
int16_t    motor2;

uint16_t   loop_counter = 0;

#define USE_UART
//#define DEBUG_GYRO

void setup() 
{
#ifdef USE_UART
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println("\n\n! eX_ESP_Robot ready !\n");
  
#endif
  WiFi_Start();
#ifdef USE_UART
  Serial.println("Wifi ready");
#endif
  te_Start();
  i2c_begin(SDA_PIN, SCL_PIN, I2C_SPEED);
  robot_pro_mode = false;
  robot_shutdown = false;
  int mpu_res = mpu_Initialization();
  Serial.print("mpu_Initialization ");
  Serial.println(mpu_res);
  timer_old_value = millis();
  fadder[0] = 0.5;
  fadder[1] = 0.5;
  fadder[2] = 0.5;
  fadder[3] = 0.5;
// Red led for debugging and to signal that the gyro did not initalize
#ifdef DEBUG_GYRO
  pinMode(LED_RED,OUTPUT);  
  digitalWrite(LED_RED,HIGH); // OFF
#endif

  digitalWrite(MOTORS_ENABLE_PIN, LOW);
  for (uint8_t k = 0; k < 5; k++) {
    te_SetMotorsSpeed(10, 10);
    te_SetServo(SERVO_AUX_NEUTRO + 3);
    delay(200);
    te_SetMotorsSpeed(-10, -10);
    te_SetServo(SERVO_AUX_NEUTRO - 3);
    delay(200);
  }
  te_SetMotorsSpeed(0, 0);
  digitalWrite(MOTORS_ENABLE_PIN, LOW);
  te_SetServo(SERVO_AUX_NEUTRO);

  setMicroStepping(MICROSTEPPING);
}

void loop()
{
  if (robot_shutdown)
    return;
  if(OSC_MSG_Read())
  {
    if (page==1)
    {
      if ((!robot_pro_mode)&&(toggle[0]==1))
      {
        // Change to PRO mode
        max_throttle = MAX_THROTTLE_PRO;
        max_steering = MAX_STEERING_PRO;
        max_target_angle = MAX_TARGET_ANGLE_PRO;
        robot_pro_mode = true;    
      }
      if ((robot_pro_mode)&&(toggle[0]==0))
      {
        // Change to NORMAL mode
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        robot_pro_mode = false;
      }
      // Push reset controls to neutral position
      if (push[1] || ChangePage)
      {
        ChangePage = false;
        fadder[0] = 0.5;
        WiFi_MSG_Send_Float("/1/fader1\0\0\0,f\0\0\0\0\0\0",20,fadder[0]);
        fadder[1] = 0.5;
        WiFi_MSG_Send_Float("/1/fader2\0\0\0,f\0\0\0\0\0\0",20,fadder[1]);
      }

      throttle = (fadder[0] - 0.5) * max_throttle;
      steering = fadder[1] - 0.5;
      if (steering > 0)
        steering = ( steering * steering + 0.5 * steering) * max_steering;
      else
        steering = (-steering * steering + 0.5 * steering) * max_steering;
    }
    else // set Page=2
    {
      throttle = 0;
      steering = 0;
      if (ChangePage)
      {
        ChangePage = false;
        fadder[0] = Kp_user/KP/2;
        WiFi_MSG_Send_Float("/2/fader1\0\0\0,f\0\0\0\0\0\0",20,fadder[0]);
        fadder[1] = Kd_user/KD/2;
        WiFi_MSG_Send_Float("/2/fader2\0\0\0,f\0\0\0\0\0\0",20,fadder[1]);
        fadder[2] = Kp_thr_user/KP_THROTTLE/2;
        WiFi_MSG_Send_Float("/2/fader3\0\0\0,f\0\0\0\0\0\0",20,fadder[2]);
        fadder[3] = Ki_thr_user/(KI_THROTTLE+0.1)/2;
        WiFi_MSG_Send_Float("/2/fader4\0\0\0,f\0\0\0\0\0\0",20,fadder[3]);
      }
      else
      {
        Kp_user = KP*2*fadder[0];
        Kd_user = KD*2*fadder[1];
        Kp_thr_user = KP_THROTTLE*2*fadder[2];
        Ki_thr_user = KI_THROTTLE*2*fadder[3];
      }
      while (toggle[1])
      {
      //Reset external parameters
        mpuResetFIFO();
        PID_errorSum = 0;
        control_output = 0;
        timer_old_value = millis(); 
        te_SetMotorsSpeed(0,0);
        OSC_MSG_Read();
      }
    }
  }
    // New DMP Orientation solution?
  fifoCount = mpuGetFIFOCount();
  if (fifoCount>=18)
  {
    if (fifoCount>18)
    {
      mpuResetFIFO();
      return;
    }
    loop_counter++;
    timer_value = millis();
    dt = (timer_value - timer_old_value);
    timer_old_value = timer_value;
    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    float angle_perceived = dmpGetPhi();
    angle_adjusted = -angle_perceived * 90/70;  // reverse angle sign and adjust proportion error
//    Serial.println(angle_adjusted);

    if ((angle_adjusted<74)&&(angle_adjusted>-74))  // is the robot in working position?
    {
      // NORMAL MODE
      PIN_LOW(MOTORS_ENABLE_PIN);    // drivers are active

      // Push1 Move servo arm
      if (push[0])  // Move arm
        te_SetServo(SERVO_MIN_PULSEWIDTH+10);
      else
        te_SetServo(SERVO_AUX_NEUTRO);
      if ((angle_adjusted<40)&&(angle_adjusted>-40))
      {
        Kp = Kp_user;            // gets the default user control
        Kd = Kd_user; 
        Kp_thr = Kp_thr_user;
        Ki_thr = Ki_thr_user;
        setMicroStepping(MICROSTEPPING);
      }     
      else    // while lifting the robot to the working position, we use special control parameters
      {
        Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
        Kd = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP; 
        Ki_thr = KI_THROTTLE_RAISEUP;
        setMicroStepping(NO_MICROSTEPPING);
      } 
    }
    else   // Robot not ready (flat), angle > 70º => ROBOT OFF
    {
      PIN_HIGH(MOTORS_ENABLE_PIN);    // drivers are not active
      PID_errorSum = 0;  // Reset PID I term
      control_output = 0;
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;

      // if we pulse push1 button we raise up the robot with the servo arm
      if (push[0])
      {
        // Because we know the robot orientation (face down of face up), we move the servo in the appropiate direction for raise up
        if (angle_adjusted>0)
          te_SetServo(SERVO_MIN_PULSEWIDTH);
        else
          te_SetServo(SERVO_MAX_PULSEWIDTH);
      }
      else
        te_SetServo(SERVO_AUX_NEUTRO);

    }

      // We calculate the design speed of the robot
      // Estimated speed = angular velocity of stepper motors (in combination) - angular velocity of the robot (angle is measured by the IMU)
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2)/2;                                           // Positive: forward
    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old)*90.0;                  // 90 empirical coefficient obtained when adjusting real indicators
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;                   // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;

      // SPEED CONTROL: This is a PI controller. 
      // input: user throttle
      // variable: estimated robot speed
      // output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr); 
    target_angle = constrain(target_angle,-max_target_angle,max_target_angle);                                   // limited output

      // Stability control: This is a PD controller. 
      // input: robot target angle(from SPEED CONTROL)
      // variable: robot angle
      // output: Motor speed
      // We integrate the reaction (summing), so at the output we have the acceleration of the motor, and not its speed of rotation.
    control_output += stabilityPDControl(dt,angle_adjusted,target_angle,Kp,Kd);  

      // Enter the user's taxi corrections into the control signal
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);   
    motor2 = constrain(motor2,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);

    setMicroStepping(abs(control_output) >= AUTO_STEPPING_SPEED ? NO_MICROSTEPPING : MICROSTEPPING);
    te_SetMotorsSpeed(motor1, motor2);  
/*
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.print(target_angle);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered / 10);
*/
  }

  if(RequestBAT)
  {
//    bat_level = ??;
    RequestBAT = false;
    if (page==1)
    {
      WiFi_MSG_Send_Float("/1/label1\0\0\0,f\0\0\0\0\0\0",20,bat_level);
      WiFi_MSG_Send_Float("/1/label2\0\0\0,f\0\0\0\0\0\0",20,dist_value);
    }
  }

  if (loop_counter > 400)
  {
    dist_value = echo_value * 0.0125 / 58; // in centimeters
    loop_counter = 0;
//    Serial.println(control_output);

      // send battery
      float bat_voltage = analogRead(BATTERY_PIN)*K_bat;
      // Serial.println(bat_voltage);
      
      int battery_mapped = constrain(map((int)(bat_voltage * 10), 60, 90, 55, 80), 55, 80);
//      Serial.print(" B");
//      Serial.println(battery_mapped);
      char auxS[25];
      sprintf(auxS, "$tB,%04d", battery_mapped);
//      OSC_MsgSend(auxS, 25);  
  }

}


