#ifndef MOTOR_MODULE_MAIN_H
#define MOTOR_MODULE_MAIN_H

#include <HardwareSerial.h>

HardwareSerial SystemUART(1);  // UART1
const int SystemUARTRX = 7;
const int SystemUARTTX = 8;
#define DEVICE_ID "1"

#define IR_STOP_FRONT_PIN 0
#define IR_STOP_SIDE_PIN 1
volatile bool IR_STOP_FRONT_Flag = false, IR_STOP_SIDE_Flag = false;
volatile unsigned long IR_STOP_FRONT_Time = 0, IR_STOP_SIDE_Time = 0;
volatile unsigned long Last_IR_STOP_FRONT_Time = 0, Last_IR_STOP_SIDE_Time = 0;

#define C1_ENCODER_PIN 20
#define C2_ENCODER_PIN 21


static TaskHandle_t Speed_Calculation_Task = NULL;
static TaskHandle_t Motor_Task = NULL;
static TaskHandle_t Displacement_Task = NULL;

hw_timer_t* timer = NULL;
uint8_t timer_id = 0;
uint16_t prescaler = 80;  // Between 0 and 65 535
int threshold = 50000;    // 64 bits value (limited to int size of 32bits) -- Set to 100ms

volatile uint8_t last_encoder_state = 0;
volatile int encoder_ticks = 0;  // Updated in ISR
int last_encoder_ticks = 0;
unsigned long last_time = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

const float ticks_per_rev = 2000.0;  // ✅ Encoder ticks per motor shaft revolution
const float ticksToRadians = 2 * PI / ticks_per_rev;
int delta_Ticks = 0;
float Current_speed = 0;

double radsSP = 0;  //ROS compliant.

float Kp = 1.1;
float Ki = 0;
float Kd = 0.04;
double motorpwm = 0;

float integral = 0;
float previous_error = 0;
unsigned long last_time_MOTOR = 0;

double LinearDistanceSP = 0;  //ROS compliant.
const float wheel_diameter_mm = 97.0;
const float wheel_circumference_mm = PI * wheel_diameter_mm;
const float ticks_per_mm = ticks_per_rev / wheel_circumference_mm;

double displacementKp = 2.0, displacementKi = 0.5, displacementKd = 0.1;
double displacement_integral = 0, last_displacement_error = 0;
double target_position = 0;  // In encoder ticks, or convert to radians


void IRAM_ATTR timer_isr();
void IRAM_ATTR encoderISR();
void TimerTask(void* parameters);

void UART_SEND(String message);
void checkUSBInput();

#include "BTS7960.h"
#include "Device_Networking.h"
#include "Movement_Control.h"
#include "BTS7960.H"

#endif