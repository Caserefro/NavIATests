#include "MOTOR_MODULE_MAIN.h"

// Pins and PWM channels
#define RPWM_PIN 5
#define LPWM_PIN 6
#define RPWM_CHANNEL 0
#define LPWM_CHANNEL 1

BTS7960Motor motor(RPWM_PIN, LPWM_PIN, RPWM_CHANNEL, LPWM_CHANNEL);

void setup() {
  Serial.begin(115200);  // USB Serial
  SystemUART.begin(115200, SERIAL_8N1, SystemUARTRX, SystemUARTTX);
  //motorSetup();
  Serial.println("I CAN PRINT ran succesfull --------------------------");
  pinMode(C1_ENCODER_PIN, INPUT_PULLDOWN);
  pinMode(C2_ENCODER_PIN, INPUT_PULLDOWN);
  attachInterrupt(C1_ENCODER_PIN, encoderISR, CHANGE);
  attachInterrupt(C2_ENCODER_PIN, encoderISR, CHANGE);

  // pinMode(10, OUTPUT);
  // pinMode(9, OUTPUT);

  pinMode(IR_STOP_FRONT_PIN, INPUT_PULLDOWN);
  pinMode(IR_STOP_SIDE_PIN, INPUT_PULLDOWN);
  attachInterrupt(IR_STOP_FRONT_PIN, IR_stop_front_ISR, RISING);
  attachInterrupt(IR_STOP_SIDE_PIN, IR_stop_side_ISR, RISING);

  xTaskCreate(SpeedCalculationTask,
              "Timer Task",
              4096,
              NULL,
              1,
              &Speed_Calculation_Task);

  xTaskCreate(MotorTask,
              "Motor Task",
              4096,
              NULL,
              1,
              &Motor_Task);
  xTaskCreate(DisplacementTask,    // Task function
              "Displacement PID",  // Name
              4096,                // Stack size in words (adjust if needed)
              NULL,                // Parameters
              1,                   // Priority (equal or lower than MotorTask)
              &Displacement_Task   // Task handle (optional)
  );


  last_encoder_state = (digitalRead(C1_ENCODER_PIN) << 1) | digitalRead(C2_ENCODER_PIN);
  /*
  timer = timerBegin(timer_id, prescaler, true);
  timerAttachInterrupt(timer, &timer_isr, true);
  timerAlarmWrite(timer, threshold, true);
  timerAlarmEnable(timer);*/
  motor.begin();
}

void loop() {
  // checkUSBInput();
  // Optional: Echo any response back from UART1 to USB
  checkSystemUART();
  if (IR_STOP_FRONT_Flag) {
    Serial.println("IR_STOP_FRONT_PIN");
    Serial.println(IR_STOP_FRONT_PIN);
    IR_STOP_FRONT_Flag = false;
  }
  if (IR_STOP_SIDE_Flag) {
    Serial.println("IR_STOP_SIDE_PIN");
    Serial.println(IR_STOP_SIDE_PIN);
    IR_STOP_SIDE_Flag = false;
  }

  String command;
  if (Serial.available()) {         // check Serial for new command
    command = Serial.readString();  // read the new command from Serial
    command.toLowerCase();          // convert it to lowercase
    if (command.startsWith("move")) {
      float tempSP = 0, tempKp = Kp, tempKi = Ki, tempKd = Kd;

      // Parse tokens
      char *token = strtok((char *)command.c_str(), " ");
      while (token != nullptr) {
        if (strcmp(token, "move") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempSP = atof(token);
        } else if (strcmp(token, "kp") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKp = atof(token);
        } else if (strcmp(token, "ki") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKi = atof(token);
        } else if (strcmp(token, "kd") == 0) {
          token = strtok(nullptr, " ");
          if (token) tempKd = atof(token);
        } else {
          token = strtok(nullptr, " ");
        }
      }

      // Apply values
      radsSP = tempSP;
      Kp = tempKp;
      Ki = tempKi;
      Kd = tempKd;

      Serial.printf("--> Setpoint: %.2f rad/s\n", radsSP);
      Serial.printf("--> PID Tuning | Kp: %.2f | Ki: %.2f | Kd: %.2f\n", Kp, Ki, Kd);
    }
  }
}
/*
void IRAM_ATTR timer_isr() {

  xTaskResumeFromISR(Speed_Calculation_Task);
  portYIELD_FROM_ISR();  // Not required if task priority is 0
}
*/

void SpeedCalculationTask(void *parameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // Run every 10ms

  while (1) {
    unsigned long now = millis();

    // Use FreeRTOS critical section (no mutex needed)
    portENTER_CRITICAL(&mux);
    delta_Ticks = encoder_ticks - last_encoder_ticks;
    last_encoder_ticks = encoder_ticks;
    portEXIT_CRITICAL(&mux);

    // Calculate time difference
    static unsigned long last_time = 0;  // Static to preserve value
    unsigned long dt = now - last_time;
    last_time = now;

    if (dt > 0) {
      portENTER_CRITICAL(&motorMux);
      Current_speed = (delta_Ticks * ticksToRadians) / (dt / 1000.0);
      portEXIT_CRITICAL(&motorMux);
    }
    // Periodic execution (replaces vTaskSuspend)
    //Serial.println(encoder_ticks);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void MotorTask(void *parameters) {
  while (1) {
    double error, derivative, response;
    unsigned long now = millis();
    double dt = (now - last_time_MOTOR) / 1000.0;

    portENTER_CRITICAL(&motorMux);
    error = radsSP - Current_speed;
    portEXIT_CRITICAL(&motorMux);

    /*
    // Deadband implementation
    if (fabs(error) < deadband) {
      error = 0;
      integral = 0;  // Optional: clear integral to avoid windup
    }
*/
    // integral += error * dt;
    derivative = (error - previous_error) / dt;

    response = Kp * error + Kd * derivative;

    previous_error = error;
    last_time_MOTOR = now;

    // Clamp response to [-255, 255]
    if (response > 100) response = 100;
    if (response < -100) response = -100;
    // Move motor using new API
    motorpwm += response;
    if (motorpwm > 100) motorpwm = 100;
    if (motorpwm < -100) motorpwm = -100;

    motor.move(motorpwm);  // Accepts negative or positive for direction
/*
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | Speed: ");
    Serial.print(Current_speed);
    Serial.print(" | Response: ");
    Serial.print(response);
    Serial.print(" | Kp * error: ");
    Serial.print(Kp * error);
    Serial.print(" | Ki * integral: ");
    Serial.print(Ki * integral);
    Serial.print(" | Kd * derivative: ");
    Serial.print(Kd * derivative);
    Serial.print(" | motorpwm: ");
    Serial.println(motorpwm);
*/
    vTaskDelay(pdMS_TO_TICKS(10));  // 50Hz update rate
  }
}

void DisplacementTask(void *parameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(30);

  while (1) {
    unsigned long now = millis();
    static unsigned long last_time = 0;
    unsigned long dt = now - last_time;
    last_time = now;

    double current_position;
    portENTER_CRITICAL(&mux);
    current_position = encoder_ticks;
    portEXIT_CRITICAL(&mux);

    double error = target_position - current_position;
    displacement_integral += error * (dt / 1000.0);
    double derivative = (error - last_displacement_error) / (dt / 1000.0);
    last_displacement_error = error;

    double output_speed = displacementKp * error + displacementKi * displacement_integral + displacementKd * derivative;

    // Stop when within 1 mm worth of ticks (you can adjust this tolerance)
    if (fabs(error) < (ticks_per_mm * 1.0)) {
      portENTER_CRITICAL(&motorMux);
      radsSP = 0;
      portEXIT_CRITICAL(&motorMux);

      displacement_integral = 0;
      last_displacement_error = 0;

      vTaskSuspend(NULL);  // Suspend this task until resumed
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void IRAM_ATTR encoderISR() {
  uint8_t curr_state = (digitalRead(C1_ENCODER_PIN) << 1) | digitalRead(C2_ENCODER_PIN);
  uint8_t transition = (last_encoder_state << 2) | curr_state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoder_ticks++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoder_ticks--;
      break;
    default:
      // Invalid or bounce — ignore
      break;
  }
  last_encoder_state = curr_state;
}
