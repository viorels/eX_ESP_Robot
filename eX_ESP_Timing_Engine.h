#ifndef _ESP_TIMING_ENGINE_H_
#define _ESP_TIMING_ENGINE_H_

//#include "eX_ESP_Config.h"


#define PIN_READ(x)           ((GPI & (1 << x)) != 0)
#define PIN_LOW(x)            (GPOC = (1 << x))
#define PIN_HIGH(x)           (GPOS = (1 << x))
#define SONAR_TRIG_PIN_LOW()  (GP16O &= ~1)
#define SONAR_TRIG_PIN_HIGH() (GP16O |= 1)

#define NUM_DEV  4
#ifdef USE_2_SERVOS
  #define NUM_DEV  5
#endif
///  Переменные подпрограмм управления моторами ///
int16_t    speed_M1, speed_M2;        // текущая скорость вращения моторов
int32_t    echo_start, echo_stop, echo_value=1000000;

typedef struct {
  int8_t         phase_or_dir;
  int32_t        a_period;
  int32_t        b_period;
  int32_t        c_period;
} device_header_t;

device_header_t  devices[NUM_DEV];


// Переопределение функций прерывания, которых нет в ранних версиях API

#define xt_rsil(level) (__extension__({uint32_t state; __asm__ __volatile__("rsil %0," __STRINGIFY(level) : "=a" (state)); state;}))
#define xt_wsr_ps(state)  __asm__ __volatile__("wsr %0,ps; isync" :: "a" (state) : "memory")
#define te_write(count) __asm__ __volatile__("wsr %0,ccompare0; esync"::"a" (count) : "memory")

uint32_t te_getCycleCount()
{
    uint32_t ccount;
    __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
    return ccount;
}

static volatile timercallback te_user_cb = NULL;

void ICACHE_RAM_ATTR te_isr_handler(void* para)
{
    if (te_user_cb) 
    {
      uint32_t savedPS = xt_rsil(15); // stop other interrupts
      te_user_cb();
      xt_wsr_ps(savedPS);   // start other interrupts
    }
}

void te_isr_init()
{
    ETS_CCOMPARE0_INTR_ATTACH(te_isr_handler, NULL);
}

void te_attachInterrupt(timercallback userFunc) {
    te_user_cb = userFunc;
    ETS_CCOMPARE0_ENABLE();
}

void te_detachInterrupt() {
    te_user_cb = NULL;
    ETS_CCOMPARE0_DISABLE();
}


void ICACHE_RAM_ATTR te_Processor()
{
uint32_t cc;
  te_write(te_getCycleCount() + 675); // задаем период в 675 тиков таймера для формирования рабочего периода в 10 мкс (340 для 5 мкс)
  PIN_LOW(MOTOR1_STEP_PIN);
  PIN_LOW(MOTOR2_STEP_PIN);
  for(int i=0; i < NUM_DEV; ++i)
  {
    if((--devices[i].a_period) == 0)
    {
      if(i == 0) // обработка шагового двигателя 1
      {
        if(devices[0].phase_or_dir < 0)
        {
          devices[0].a_period = devices[0].c_period;
        }
        else 
        {
          devices[0].a_period = devices[0].b_period;
          if(devices[0].phase_or_dir == !PIN_READ(MOTORS_DIR_PIN))
          {
            if(devices[0].phase_or_dir)
            {
              PIN_HIGH(MOTORS_DIR_PIN);             // DIR Motors for Motor1 (Forward)
            }
            else
            {
              PIN_LOW(MOTORS_DIR_PIN);              // DIR Motors for Motor1 (Revers)
            }
            cc = te_getCycleCount();
            while((te_getCycleCount() - cc) < PRE_DIR_STROBE);
          }
          PIN_HIGH(MOTOR1_STEP_PIN);
        }
      }
      else if(i == 1) // обработка шагового двигателя 2
      {
        if(devices[1].phase_or_dir < 0)
        {
          devices[1].a_period = devices[1].c_period;
        }
        else 
        {
          devices[1].a_period = devices[1].b_period;
          if(devices[1].phase_or_dir == PIN_READ(MOTORS_DIR_PIN))
          {
            if (devices[1].phase_or_dir)
            {
              PIN_LOW(MOTORS_DIR_PIN);                     // DIR Motors for Motor2 (Revers)
            }
            else
            {
              PIN_HIGH(MOTORS_DIR_PIN);                    // DIR Motors for Motor2 (Forward)
            }
            cc = te_getCycleCount();
            while((te_getCycleCount() - cc) < PRE_DIR_STROBE);
          }
          PIN_HIGH(MOTOR2_STEP_PIN);
        }
      }
      else if(i == 2) // обработка запускающего импульса сонара
      {
        if(devices[2].phase_or_dir)
        {
          SONAR_TRIG_PIN_LOW();
          devices[2].phase_or_dir = 0;
          devices[2].a_period = devices[2].b_period - 1;
        }
        else
        {
          SONAR_TRIG_PIN_HIGH();
          devices[2].phase_or_dir = 1;
          devices[2].a_period = 1;    // длительность положительного строба 10 мкс или 1 период управления
        }
      }
      else if(i == 3) // обработка серво-привода 1
      {
        if(devices[3].phase_or_dir)
        {
          PIN_LOW(SERVO1_PIN);
          devices[3].phase_or_dir = 0;
          devices[3].a_period = devices[3].c_period - devices[3].b_period;
        }
        else
        {
          PIN_HIGH(SERVO1_PIN);
          devices[3].phase_or_dir = 1;
          devices[3].a_period = devices[3].b_period;    // длительность положительного строба 
        }
      }
#ifdef USE_2_SERVOS
      else if(i == 4) // обработка серво-привода 2
      {
        if(devices[4].phase_or_dir)
        {
          PIN_LOW(SERVO2_PIN);
          devices[4].phase_or_dir = 0;
          devices[4].a_period = devices[3].c_period - devices[3].b_period;
        }
        else
        {
          PIN_HIGH(SERVO2_PIN);
          devices[4].phase_or_dir = 1;
          devices[4].a_period = devices[3].b_period;    // длительность положительного строба 
        }
      }
#endif
    }
  }
}

void ICACHE_RAM_ATTR te_SonarEcho()
{
  if(PIN_READ(SONAR_ECHO_PIN))
  {
    echo_start = te_getCycleCount();
  }
  else
  {
    echo_stop = te_getCycleCount();
    echo_value = echo_stop - echo_start;
  }
}

void te_Start()
{
// Инициируем выходы управления драйверами моторов
  pinMode(MOTORS_ENABLE_PIN,OUTPUT);
  PIN_LOW(MOTORS_ENABLE_PIN);    // драйвера активны
  pinMode(MOTORS_DIR_PIN,OUTPUT);
  
// Инициируем motor1 как devices[0]
  pinMode(MOTOR1_STEP_PIN,OUTPUT);
  PIN_LOW(MOTOR1_STEP_PIN);
  devices[0].phase_or_dir = -1;  // -1 - шаг не делать, 0 - шаг назад, 1 - шаг вперёд
  devices[0].a_period = 1;
  devices[0].b_period = ZERO_SPEED;
  devices[0].c_period = ZERO_SPEED;
  
// Инициируем motor2 как devices[1]
  pinMode(MOTOR2_STEP_PIN,OUTPUT);
  PIN_LOW(MOTOR2_STEP_PIN);
  devices[1].phase_or_dir = -1;  // -1 - шаг не делать, 1 - шаг назад, 0 - шаг вперёд (инверсия стгналов управления)
  devices[1].a_period = 1;
  devices[1].b_period = ZERO_SPEED;
  devices[1].c_period = ZERO_SPEED;
  
// Инициируем sonar как devices[2]
  pinMode(SONAR_TRIG_PIN,OUTPUT);
  SONAR_TRIG_PIN_LOW();
  pinMode(SONAR_ECHO_PIN,INPUT);
  attachInterrupt(SONAR_ECHO_PIN, te_SonarEcho, CHANGE);
  devices[2].phase_or_dir = 0;
  devices[2].a_period = 1;
  devices[2].b_period = 4000;    // максимальное значение периода 40 миллисекунд или при длительности периода обработки 10 мкc  - 4000 периодов
// Инициируем servo1 как devices[3]
  pinMode(SERVO1_PIN,OUTPUT);
  PIN_LOW(SERVO1_PIN);
  devices[3].phase_or_dir = 0;
  devices[3].a_period = 1;
  devices[3].b_period = SERVO_AUX_NEUTRO; 
  devices[3].c_period = 2000;  
#ifdef USE_2_SERVOS
  // Инициируем servo2 как devices[4]
  pinMode(SERVO2_PIN,OUTPUT);
  PIN_LOW(SERVO2_PIN);
  devices[4].phase_or_dir = 0;
  devices[4].a_period = 1;
  devices[4].b_period = SERVO_AUX_NEUTRO; 
  devices[4].c_period = 2000; 
#endif
// Запуск таймера
  te_isr_init();
  te_attachInterrupt(te_Processor);
  te_write(te_getCycleCount() + 1500);
}

// Установка скорости вращения моторов
// tspeed_Mx может принимать как положительные, так и отрицательные значения (reverse)
void te_SetMotorsSpeed(int16_t t_speed_M1, int16_t t_speed_M2)
{ 
  int32_t   speed_1, speed_2;
  // Лимитируем изменение скорости с учётом максимального ускорения для motor1
  if ((speed_M1 - t_speed_M1) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - t_speed_M1) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = t_speed_M1;
    
  // Лимитируем изменение скорости с учётом максимального ускорения для motor2
  if ((speed_M2 - t_speed_M2) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - t_speed_M2) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = t_speed_M2;

  speed_1 = speed_M1 * K_MOTOR_SPEED;
  speed_2 = speed_M2 * K_MOTOR_SPEED;
// Расчёт периода для motor1
  if (speed_1 == 0)
  {
    devices[0].b_period = ZERO_SPEED;
    devices[0].phase_or_dir = -1;
  }
  else 
    if (speed_1 > 0)
    {
      devices[0].b_period = 6400000/speed_1;   // 6400000 - это максимальное значение speed_M1 (max. 500) * K_MOTOR_SPEED (max.12800)
      devices[0].phase_or_dir = 1;
    }
    else
    {
      devices[0].b_period = 6400000/-speed_1;
      devices[0].phase_or_dir = 0;
    }

// Расчёт периода для motor2
  if (speed_2 == 0)
  {
    devices[1].b_period = ZERO_SPEED;
    devices[1].phase_or_dir = -1;
  }
  else
    if (speed_2 > 0)
    {
      devices[1].b_period = 6400000/speed_2;
      devices[1].phase_or_dir = 1;
    }
    else 
    {
      devices[1].b_period = 6400000/-speed_2;
      devices[1].phase_or_dir = 0;
    }
}
void te_SetServo(int pwm, int num=0)
{
  if (num == 0)
    devices[3].b_period = pwm;
#ifdef USE_2_SERVOS
  if (num == 1)
    devices[4].b_period = pwm;
#endif
}
#endif // _ESP_TIMING_ENGINE_H_
