/* * * * * * * * * * * * * * * * * * * * * * *
 * Mechatronics system design project 
 *
 * Code by: Mina Samer
 * Date :10/5/2024
 * * * * * * * * * * * * * * * * * * * * * * */

/*********************************************************************
 *                            libraries                              *
 *********************************************************************/
//for including servo libirary for using servo
#include <Servo.h>

//for including atomic libirary for pid
#include <util/atomic.h>

/*********************************************************************
 *                            definition                             *
 *********************************************************************/
//pid on & off
#define PID_ON
//#define PID_OFF

// encoder Pins
#define ENCA 2
#define ENCB 3

//motor pins
#define PWM 5
#define IN1 6
#define IN2 7

// ir pins
#define Px1 8
#define Px2 13

//servo pins
#define SERVO_SIGNAL 10

//communication constant
#define SIG_1 A0
#define SIG_2 A1
#define SIG_3 A2

// product constants
#define NOTHING 0
#define BASE 1
#define LED 2
#define BLUE 3
#define GREEN 4

//cam constants
#define CAM_BLUE '1'
#define CAM_GREEN '2'

// dc motor constants
#define FOW -1
#define BACK 1
#define STOP 0
#define WORK_SPEED 190
#define ARM_WORKING_DELAY 10000

// servo constants
#define SERVO_START 20
#define SERVO_END 170
#define SERVO_DELAY 2000

// pid constants
#define KP 1.65
#define KI 0.015
#define KD 0.225
#define TARGET 1480
#define PWMMAX 255
#define PID_DELAY 1

//ir constant
#define OBJ 0
#define NOOBJ 1
#define IR_DELAY 250

//error constant
#define DURATION 15000

/*********************************************************************
 *                         function prtotype                         *
 *********************************************************************/
void arm_Conect(void);
void servo(void);

/*********************************************************************
 *                         creating objects                          *
 *********************************************************************/
Servo s1;

/*********************************************************************
 *                         global variables                          *
 *********************************************************************/
//pid globals
volatile int posi = LOW;
long prevT = LOW;
float eprev = LOW;
float eintegral = LOW;

long x = LOW;

// checking flags
char pid_flag = LOW;    //pid flage to solve restarting problem
char servo_flag = LOW;  //for servo opening and closing loop
char arm_flag = LOW;    //for arm loop

/*********************************************************************
 *                         Initialization                            *
 *********************************************************************/
void setup() {

  //giving servo signal pins configration
  s1.attach(SERVO_SIGNAL);

  //ir pins configration
  pinMode(Px1, INPUT);  //lower  ir
  pinMode(Px2, INPUT);  //higher ir

  //communications pins configration
  pinMode(SIG_1, OUTPUT);
  pinMode(SIG_2, OUTPUT);
  pinMode(SIG_3, OUTPUT);

  //encoder pins configration
  pinMode(ENCA, INPUT);  //encoder phase a
  pinMode(ENCB, INPUT);  //encoder phase b

  //motor pins configration
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //using interrupt to get readings from encoder
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  servo();

  //strting with baud rate 9600
  Serial.begin(9600);
}

/*********************************************************************
 *                         program loop                              *
 *********************************************************************/
void loop() {

  arm_Conect();
}

/*********************************************************************
 *                         functions                                 *
 *********************************************************************/
//makes open & close sevro
void servo(void) {
  s1.write(SERVO_END);    //makes servo open
  delay(SERVO_DELAY);     //delay to makes servo takes its time
  s1.write(SERVO_START);  //return servo to its postion
}

//rest pid values for reusing it after stoping the motor
void pid_Reset(void) {
  volatile int posi = LOW;
  long prevT = LOW;
  float eprev = LOW;
  float eintegral = LOW;
}

//for seting motor direction and speed
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // Motor speed
  if (dir == BACK) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == FOW) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

//for taking encoder reading for pid calc
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > LOW) {
    posi++;
  } else {
    posi--;
  }
}

//pid calc for motor to call it when we need
void motor(void) {

  // set target position
  int target = TARGET;
  // int target = 250*sin(prevT/1e6);

  // PID constants kp=1.65, kd=0.23 ki=0.015
  float kp = KP;
  float ki = KI;
  float kd = KD;

  // time difference
  long currT = micros() - x;
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = LOW;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // Set the motor speed and direction
  int dir = FOW;
  if (u < LOW) {
    dir = BACK;
  }
  float pwr = fabs(u);
  if (pwr > PWMMAX) {
    pwr = PWMMAX;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);
  delay(PID_DELAY);

  // store previous error
  eprev = e;

  pid_flag = HIGH;
}

//for taking readings from cam and return color to arduino
char cam_Read(void) {
  //checking for serial signal
  if (Serial.available() > LOW) {
    switch (Serial.read()) {  //switch cases to diff between cam readings
      case CAM_BLUE:          //blue reading
        return BLUE;          //return blue to as feedback
        break;
      case CAM_GREEN:  //green reading
        return GREEN;  //return green as feedback
        break;
      default:
        return NOTHING;  //return  nothing fe their is no color detected
        break;
    }
  }
}
//take ir signal and return the type of object
char ir_Read(void) {
  //reading ir  signals
  int val1 = digitalRead(Px1);
  int val2 = digitalRead(Px2);

  //TESTING PART
  // Serial.println("LOW");
  // Serial.println(val1);
  // Serial.println("HIGH");
  // Serial.println(val2);
  // delay(5000);

  //checking ir readings
  //if no object detected
  if ((val1 == NOOBJ) && (val2 == NOOBJ)) {
    unsigned short startTime = millis();  // Record the start time
    servo_flag = LOW;                     //close servo flage
    if (arm_flag == HIGH) {               //checking if arm flag is high
      delay(ARM_WORKING_DELAY);           //give delay to make gripper takes it time
    }
    //for pid working or closing
#ifdef PID_ON
    pid_Reset();  //rest pid values
    //checking for pid flag
    if (pid_flag == HIGH) {
      x = micros();
    }
#endif
    //loop for motor untill pid stops it or irs
    while ((val1 == NOOBJ) && (val2 == NOOBJ)) {
      val1 = digitalRead(Px1);
      val2 = digitalRead(Px2);
      if ((val1 == NOOBJ) && (val2 == NOOBJ)) {
        arm_Signal(0, 0, 0);
      }
#ifdef PID_ON
      motor();
#endif
//
#ifdef PID_OFF
      setMotor(FOW, WORK_SPEED, PWM, IN1, IN2);
#endif
      unsigned short currentTime = millis();                 // Get the current time
      unsigned short elapsedTime = currentTime - startTime;  // Calculate the elapsed time
      if (elapsedTime >= DURATION) {                         //timer for servo and pid errors
        servo();
        while ((val1 == NOOBJ) && (val2 == NOOBJ)) {
          val1 = digitalRead(Px1);
          val2 = digitalRead(Px2);
          setMotor(FOW, WORK_SPEED, PWM, IN1, IN2);
        }
      }
    }
    return NOTHING;
  } else if ((val1 == OBJ) && (val2 == NOOBJ)) {
    delay(IR_DELAY);
    setMotor(STOP, STOP, PWM, IN1, IN2);
    if (servo_flag == LOW) {
      servo();
      servo_flag = HIGH;
    }
    arm_flag = HIGH;
    //Serial.println("led");
    return LED;
  } else if ((val1 == OBJ) && (val2 == OBJ)) {
    delay(IR_DELAY);
    setMotor(STOP, STOP, PWM, IN1, IN2);
    if (servo_flag == LOW) {
      servo();
      servo_flag = HIGH;
    }
    arm_flag = HIGH;
    //Serial.println("base");
    return BASE;
  }
}

// write on gpio pins to communicate
void arm_Signal(char frist, char second, char third) {
  digitalWrite(SIG_1, frist);
  digitalWrite(SIG_2, second);
  digitalWrite(SIG_3, third);
}

// take readings for previous function and take control to arm
void arm_Conect(void) {
  char obj_Type = ir_Read();
  char obj_Color = cam_Read();
  if ((obj_Type == BASE) && (obj_Color == GREEN)) {
    arm_Signal(1, 1, 0);
  } else if ((obj_Type == BASE) && (obj_Color == BLUE)) {
    arm_Signal(0, 1, 1);
  } else if ((obj_Type == LED) && (obj_Color == GREEN)) {
    arm_Signal(1, 0, 1);
  } else if ((obj_Type == LED) && (obj_Color == BLUE)) {
    arm_Signal(1, 1, 1);
  } else if ((obj_Type == NOTHING) || (obj_Color == NOTHING)) {
    arm_Signal(0, 0, 0);
  }
}