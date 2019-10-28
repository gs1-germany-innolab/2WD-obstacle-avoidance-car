#include <analogWrite.h>

int inputPin = A6; // ultrasonic module   ECHO to A1
int outputPin = A7; // ultrasonic module  TRIG to A0
#define Lpwm_pin  33    //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  32    //pin of controlling speed---- ENB of motor driver board
//int Lpwm_pin = 33;
//int Rpwm_pin = 32;
int pinLB = 13;           //pin of controlling turning---- IN1 of motor driver board
int pinLF = 12;           //pin of controlling turning---- IN2 of motor driver board
int pinRB = 14;          //pin of controlling turning---- IN3 of motor driver board
int pinRF = 27;          //pin of controlling turning---- IN4 of motor driver board
unsigned char Lpwm_val = 105; //initialized left wheel speed at 250
unsigned char Rpwm_val = 85; //initialized right wheel speed at 250
int Car_state = 0;           //the working state of car
int servopin = 19;            //defining digital port pin 3, connecting to signal line of servo motor
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao = 90;  //initialized angle of motor at 60°

int angleServoLeft = 35;
int angleServoStraight = 0;
int angleServoRight = -35;

void servopulse(int servopin, int myangle) //defining a function of pulse
{
  myangle = max(min(myangle, 90), -90);
  pulsewidth = ((myangle + 90) * 11) + 500; //converting angle into pulse width value at 500-2480
  digitalWrite(servopin, HIGH); //increasing the level of motor interface to upmost
  delayMicroseconds(pulsewidth); //delaying microsecond of pulse width value
  digitalWrite(servopin, LOW); //decreasing the level of motor interface to the least
  delay(20 - pulsewidth / 1000);
}
void Set_servopulse(int set_val)
{
  for (int i = 0; i <= 10; i++) //giving motor enough time to turn to assigning point
    servopulse(servopin, set_val); //invokimg pulse function
}
void M_Control_IO_config(void)
{
  pinMode(pinLB, OUTPUT); // /pin 2
  pinMode(pinLF, OUTPUT); // pin 4
  pinMode(pinRB, OUTPUT); // pin 7
  pinMode(pinRF, OUTPUT); // pin 8
  pinMode(Lpwm_pin, OUTPUT); // pin 11 (PWM)
  pinMode(Rpwm_pin, OUTPUT); // pin10(PWM)
}
void Set_Speed(unsigned char Left, unsigned char Right) //function of setting speed
{
  analogWrite(Lpwm_pin, Left);
  analogWrite(Rpwm_pin, Right);
}
void advance()    //  going forward
{
  digitalWrite(pinRB, HIGH); // making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH); // making motor move towards left rear
  digitalWrite(pinLF, LOW);
  Car_state = 1;
}
void turnR()        //turning right(dual wheel)
{
  digitalWrite(pinRB, LOW); //making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW); //making motor move towards left front
  Car_state = 4;
}
void turnL()         //turning left(dual wheel)
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW );  //making motor move towards right front
  digitalWrite(pinLB, LOW);  //making motor move towards left rear
  digitalWrite(pinLF, HIGH);
  Car_state = 3;
}
void stopp()        //stop
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  Car_state = 5;
}
void back()         //back up
{
  digitalWrite(pinRB, LOW); //making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW); //making motor move towards left rear
  digitalWrite(pinLF, HIGH);
  Car_state = 2;
}



void Self_Control(void)//self-going, ultrasonic obstacle avoidance
{
  int H;

  Set_servopulse(angleServoStraight);

/*
  H = Ultrasonic_Ranging(1);
  delay(300);
  
  if (H < 35)
  {
    stopp();
    delay(100);
    back();
    delay(50);
  }
*/

  H = Ultrasonic_Ranging(1);
  if (H < 6000000000)
  {
    stopp();
    delay(100);
    Set_servopulse(angleServoLeft);
    unsigned long L = ask_pin_L(2);
    delay(300);
    Set_servopulse(angleServoRight);
    unsigned long R = ask_pin_R(3);
    delay(300);

    if (L > 60 & R > 60 & H > 35)
    {
      Set_servopulse(angleServoStraight);
      advance();
      delay(400);
    }
    else if ((L  < 35 && H < 35) || (R < 35 && H < 35))
    {
      Set_servopulse(angleServoStraight);
      stopp();
      delay(50);
      back();
      delay(50);
    }
    else if (L > R)
    {
      Set_servopulse(angleServoLeft);
      // back();
      // delay(100); //old 100
      turnL();
      delay(400);  //old 400
      stopp();
      delay(50);
      Set_servopulse(angleServoStraight);
      // H = Ultrasonic_Ranging(1);
      // delay(500);
    }
    else
    {
      Set_servopulse(angleServoRight  );
      // back();
      // delay(100);
      turnR();
      delay(400);
      stopp();
      delay(50);
      Set_servopulse(angleServoStraight);
      // H = Ultrasonic_Ranging(1);
      // delay(500);
    }

  }
  else
  {
    advance();
  }
}
int Ultrasonic_Ranging(unsigned char Mode)//function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation

{
  int old_distance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  unsigned long distance = pulseIn(inputPin, HIGH);  // reading the duration of high level
  distance = distance / 58; // Transform pulse time to distance
  if (Mode == 1) {
    Serial.print("\n H = ");
    Serial.print(distance, DEC);
    }
  return distance;
}
unsigned long ask_pin_L(unsigned char Mode)
{
  int old_Ldistance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  unsigned long Ldistance = pulseIn(inputPin, HIGH);
  Ldistance = Ldistance / 58; // Transform pulse time to distance
  if (Mode == 2) {
    Serial.print("\n L = ");
    Serial.print(Ldistance, DEC);
  }
  return Ldistance;
}
unsigned long ask_pin_R(unsigned char Mode)
{
  int old_Rdistance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); //
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  unsigned long Rdistance = pulseIn(inputPin, HIGH);
  Rdistance = Rdistance / 58; // Transform pulse time to distance
  if (Mode == 3) {
    Serial.print("\n R = ");
    Serial.print(Rdistance, DEC);
  }
  return Rdistance;
}

void setup()
{
  pinMode(servopin, OUTPUT); //setting motor interface as output
  M_Control_IO_config();     //motor controlling the initialization of IO
  Set_Speed(Lpwm_val, Rpwm_val); //setting initialized speed
  Set_servopulse(DuoJiao);       //setting initialized motor angle
  pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
  pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
  Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud
  stopp();                       //stop
}
void loop()
{
  /*
    stopp();
    delay(500);
    turnR();
    delay(500);
    turnL();
    delay(500);
  */
  Self_Control();
}
