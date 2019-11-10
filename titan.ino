#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

int SENSITIVITY = 10; // MAX 5 to MIN 150


uint8_t servonum = 12; //12 горизонт 13 вертикаль

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void turnOnAngl(int angl, int pin){
  int cur_len = map(angl, 1, 181, 50, 600);
    pwm.setPWM(pin,0, cur_len);
}
float volts = 0;
float distance = 0;

int getLevelSignal(){
 sensorValue = analogRead(analogInPin);
 volts = 1.0 * sensorValue / 1024 * 5;
 distance = 20 * pow(volts, -1);
 return (int)distance;
}

int i = 0;
void rotationY(){
    for(i = 0; i<180;i++){
    delay(1);
    Serial.print(getLevelSignal());
    Serial.print(",");
    turnOnAngl(i,12);
  }
  Serial.println();
    for(i = 180; i>0;i--){
    delay(1);
    Serial.print(getLevelSignal());
    Serial.print(",");
    turnOnAngl(i,13);
  }
  Serial.println();
}

void rotationX(){
    for(i = 0; i<180;i++){
    delay(1);
    Serial.print(i);
    Serial.print(":");
    Serial.println(getLevelSignal());
//    if(i%60==0){
//      turnOnAngl(i-90,13);
//    }
    turnOnAngl(i,12);
  }
  Serial.println();
    for(i = 180; i>0;i--){
    delay(1);
    Serial.print(i);
    Serial.print(":");
    Serial.println(getLevelSignal());
//    if(i%60==0){
//     turnOnAngl(i+90,13); 
//    }
    turnOnAngl(i,12);
  }
  Serial.println();
}

int findDirection(){
  int j = 0;
   for(j = 0; j<180;j++){
    delay(10);
    turnOnAngl(j,12);
    int d = getLevelSignal();
    Serial.println(d);
    if(d>SENSITIVITY){
      return d;
    }
  }
   for(j = 180; j>0;j--){
    delay(10);
    turnOnAngl(j,12);
    int d = getLevelSignal();
    Serial.println(d);
    if(d>SENSITIVITY){
      return d;
    }
  }
}

void loop() {
  turnOnAngl(0,12);
  Serial.println(getLevelSignal());
  if(getLevelSignal()<=SENSITIVITY){
   findDirection(); 
  }
}
