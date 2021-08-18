// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

#include <Wire.h>

bool received = false;

byte i2c_command = 0x00;
byte i2c_value = 0x00;

#define MOTOR_DIREC_S 0x00
#define MOTOR_DIREC_B 0x01
#define MOTOR_DIREC_F 0x02
#define MOTOR_DIREC_O 0x03

#define MOTOR_1_SPEED 0x30
#define MOTOR_1_DIREC 0x31
#define MOTOR_2_SPEED 0x35
#define MOTOR_2_DIREC 0x36

#define TUNE_KP       0x40
#define TUNE_KI       0x41

#define InA1            7                       // INA motor pin
#define InB1            8                       // INB motor pin
#define PWM1            9                       // PWM motor pin
#define InA2            5                       // INA motor pin
#define InB2            6                       // INB motor pin
#define PWM2            10                      // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinA2      3                       // encoder B pin

#define LOOPTIME        5                     // PID loop time

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req1 = 0;                            // speed (Set Point)
int speed_act1 = 0;                              // speed (actual value)
int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count1 = 0;                        // rev counter
static long countAnt1 = 0;
float pidTerm1 = 0;                                                            // PID correction
int error1 = 0;
static int last_error1 = 0;
int speed_req2 = 0;                            // speed (Set Point)
int speed_act2 = 0;                              // speed (actual value)
int PWM_val2 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count2 = 0;                        // rev counter
static long countAnt2 = 0;
float pidTerm2 = 0;                                                            // PID correction
int error2 = 0;
static int last_error2 = 0;
float Kp =   .9;                                // PID proportional control Gain  .4  //.9
float Ki =    1.4;                                // PID integral control gain       1


void setup() {
  Serial.begin(115200);
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT);
  digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinA2, HIGH);
  attachInterrupt(digitalPinToInterrupt(encodPinA1), encoder_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(encodPinA2), encoder_2, FALLING);

  analogWrite(PWM1, PWM_val1);
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, LOW);
  analogWrite(PWM2, PWM_val2);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, LOW);

  Wire.begin(0x08);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent);    // register event
}

void loop() {
  if ((millis() - lastMilli) >= LOOPTIME)   {                                 // enter tmed loop
    lastMilli = millis();
    getMotorData();                                                           // calculate speed, volts and Amps
    PWM_val1 = updatePid1(PWM_val1, speed_req1, speed_act1);           // compute PWM value
    analogWrite(PWM1, PWM_val1);                                              // send PWM to motor
    PWM_val2 = updatePid2(PWM_val2, speed_req2, speed_act2);           // compute PWM value
    analogWrite(PWM2, PWM_val2);                                              // send PWM to motor
  }

  if (received)
  {
    switch (i2c_command) {
      case MOTOR_1_SPEED:
        speed_req1 = int(i2c_value);
        PWM_val1 = updatePid1(PWM_val1, speed_req1, speed_act1);                       // compute PWM value
        analogWrite(PWM1, PWM_val1);                                                         // send PWM to motor
        break;
      case MOTOR_1_DIREC:
        switch (i2c_value) {
          case MOTOR_DIREC_S:
            digitalWrite(InA1, HIGH);
            digitalWrite(InB1, HIGH);
            break;
          case MOTOR_DIREC_B:
            digitalWrite(InA1, LOW);
            digitalWrite(InB1, HIGH);
            break;
          case MOTOR_DIREC_F:
            digitalWrite(InA1, HIGH);
            digitalWrite(InB1, LOW);
            break;
          case MOTOR_DIREC_O:
            digitalWrite(InA1, LOW);
            digitalWrite(InB1, LOW);
            break;
        }
        break;

      case MOTOR_2_SPEED:
        speed_req2 = int(i2c_value);
        PWM_val2 = updatePid2(PWM_val2, speed_req2, speed_act2);                       // compute PWM value
        analogWrite(PWM2, PWM_val2);                                                         // send PWM to motor
        break;
      case MOTOR_2_DIREC:
        switch (i2c_value) {
          case MOTOR_DIREC_S:
            digitalWrite(InA2, HIGH);
            digitalWrite(InB2, HIGH);
            break;
          case MOTOR_DIREC_B:
            digitalWrite(InA2, LOW);
            digitalWrite(InB2, HIGH);
            break;
          case MOTOR_DIREC_F:
            digitalWrite(InA2, HIGH);
            digitalWrite(InB2, LOW);
            break;
          case MOTOR_DIREC_O:
            digitalWrite(InA2, LOW);
            digitalWrite(InB2, LOW);
            break;
        }
        break;

      case TUNE_KP:
        Serial.print("Old Kp: "); Serial.print(Kp);
        Kp = float(int(i2c_value) / 100);
        Serial.print("\tNew Kp: "); Serial.println(Kp);
        break;
        
      case TUNE_KI:
        Serial.print("Old Ki: "); Serial.print(Ki);
        Ki = float(int(i2c_value) / 100);
        Serial.print("\tNew Ki: "); Serial.println(Ki);
        break;
        

    }
    received = false;
  }

  printMotorInfo();                                                           // display data
}

void getMotorData()  {                                                        // calculate speed, volts and Amps
  speed_act1 = ((count1 - countAnt1) * (60 * (1000 / LOOPTIME))) / 360;  // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  speed_act2 = ((count2 - countAnt2) * (60 * (1000 / LOOPTIME))) / 360;  // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  countAnt1 = count1;
  countAnt2 = count2;
}

int updatePid1(int command, int targetValue, int currentValue)   {             // compute PWM value
  error1 = abs(targetValue) - abs(currentValue);
  pidTerm1 = (Kp * error1) + (Ki * (error1 - last_error1));
  last_error1 = error1;
  return constrain(command + int(pidTerm1), 0, 255);
}

int updatePid2(int command, int targetValue, int currentValue)   {             // compute PWM value
  error2 = abs(targetValue) - abs(currentValue);
  pidTerm2 = (Kp * error2) + (Ki * (error2 - last_error2));
  last_error2 = error2;
  return constrain(command + int(pidTerm2), 0, 255);
}

void printMotorInfo()  {                                                      // display data
  if ((millis() - lastMilliPrint) >= 100 /*&& Serial.available()*/)   {
    lastMilliPrint = millis();
    Serial.print("1: SP:");             Serial.print(speed_req1);
    Serial.print("  RPM:");          Serial.print(speed_act1);
    Serial.print("  PWM:");          Serial.print(PWM_val1);
    Serial.print("\t2: SP:");             Serial.print(speed_req2);
    Serial.print("  RPM:");          Serial.print(speed_act2);
    Serial.print("  PWM:");          Serial.print(PWM_val2);

    Serial.println(".");
  }
}

void encoder_1()  {                                    // pulse and direction, direct port reading to save cycles
  count1 ++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
}
void encoder_2()  {                                    // pulse and direction, direct port reading to save cycles
  count2 ++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
}

void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    i2c_command = Wire.read(); // receive byte as a character
  }
  i2c_value = Wire.read();    // receive byte as an integer
  received = true;
}
