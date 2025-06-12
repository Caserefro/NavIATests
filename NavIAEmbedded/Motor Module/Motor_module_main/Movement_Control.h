#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H



void IRAM_ATTR IR_stop_front_ISR() {
  IR_STOP_FRONT_Time = millis();
  if (IR_STOP_FRONT_Time - Last_IR_STOP_FRONT_Time > 250) {
    IR_STOP_FRONT_Flag = true; //Maybe notifying a task? 
  }
  Last_IR_STOP_FRONT_Time = IR_STOP_FRONT_Time;
}

void IRAM_ATTR IR_stop_side_ISR() {
  IR_STOP_SIDE_Time = millis();
  if (IR_STOP_SIDE_Time - Last_IR_STOP_SIDE_Time > 250) {
    IR_STOP_SIDE_Flag = true; //Maybe notifying a task? 
  }
  Last_IR_STOP_SIDE_Time = IR_STOP_SIDE_Time;
}



#endif