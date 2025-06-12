#ifndef BTS7960_H
#define BTS7960_H


class BTS7960Motor {
public:
  BTS7960Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t rpwmChannel, uint8_t lpwmChannel, uint16_t freq = 20000, uint8_t resolution = 8)
    : _rpwmPin(rpwmPin), _lpwmPin(lpwmPin),
      _rpwmChannel(rpwmChannel), _lpwmChannel(lpwmChannel),
      _freq(freq), _resolution(resolution) {}

  void begin() {
    ledcSetup(_rpwmChannel, _freq, _resolution);
    ledcSetup(_lpwmChannel, _freq, _resolution);
    ledcAttachPin(_rpwmPin, _rpwmChannel);
    ledcAttachPin(_lpwmPin, _lpwmChannel);
    move(0.0);
  }

  void move(int speed) {
    speed = constrain(speed, -100, 100);              // Clamp to valid range
    uint8_t pwmValue = (uint8_t)(abs(speed) * 2.55);  // scale to 0–255

    if (speed > 0) {
      ledcWrite(_rpwmChannel, 0);
      ledcWrite(_lpwmChannel, pwmValue);
    } else if (speed < 0) {
      ledcWrite(_lpwmChannel, 0);
      ledcWrite(_rpwmChannel, pwmValue);
    } else {
      ledcWrite(_rpwmChannel, 0);
      ledcWrite(_lpwmChannel, 0);
    }
  }

private:
  uint8_t _rpwmPin, _lpwmPin;
  uint8_t _rpwmChannel, _lpwmChannel;
  uint16_t _freq;
  uint8_t _resolution;
};


// NOT BTS7960 CODE, MADE FOR TESTS WITH DRV8833
/*
void motorSetup() {
  // Set all the motor control inputs to OUTPUT
//  pinMode(LMIN1_PIN, OUTPUT);
//  pinMode(LMIN2_PIN, OUTPUT);
  // Turn off motors - Initial state
  digitalWrite(LMIN1_PIN, LOW);
  digitalWrite(LMIN2_PIN, LOW);

}

void Move_L_Motor(double pwm) {  //Rotation -255 - 255
  if (pwm > 255)
    pwm = 255;
  if (pwm < 0)
    pwm = 0;
  if (pwm < 0) {
    analogWrite(LMIN2_PIN, -pwm);
    digitalWrite(LMIN1_PIN, LOW);

  } else {  // stop or forward
    digitalWrite(LMIN2_PIN, LOW);
    analogWrite(LMIN1_PIN, pwm);
  }

*/
#endif