/*
   Set Board: Tools --> Board --> Arduino Mega2560
   Set Processor: Tools --> Processor --> ATmega2560
*/


#include <Servo.h>
#include "configuration.h"
#include <Wire.h>
#include <MechaQMC5883.h>
//#include <Filters.h>

///////////////////////////////////////////////////////////////////////////////
// 2.59( kp = 0.16, kd = 0.0002, Speed = 180) OK
// 2.22 (kp = 0.023, kd = 0.002, speed = 250) Blue Wrong 


///////////////////////////////////////////////////////////////////////////////
  
  MechaQMC5883 qmc;
  //PD value
  const float Kp = 0.01;//.14
  const float Kd = 0.00;///.0008/  >>
  const float SpeedAuto = 50;
  
  float lzeta = 0;
  float delta = 0;
  //................Function Declaration..............................
  int OutputToMotor1(int value);
  int OutputToMotor4(int value);
    
  //..Magnetic parameter
  int x,y,z; double zeta, magnitude;
///////////////////////////////////////////////////////////////////////////////


//=============== Function Name Declaration ===============
double Deadband(double value, double limit);
int OutputToMotor1(int value);
int OutputToMotor2(int value);
int OutputToMotor3(int value);
int OutputToMotor4(int value);



//=============== Parameters Declaration ==================
  float K     = 0.2;
  float Zeta  = 0.5;


// Type Servo
Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myServo4;

// Time
unsigned long previousLoopTime = 0;
unsigned long loopTime = 0;

// Input Signal from Receiver
int input1 = 0;
int input2 = 0;
int input3 = 0;
int input4 = 0;
int input5 = 0;
int input6 = 0;

// Output Signal to Motor Driver
int out1 = 0;
int out2 = 0;
int out3 = 0;
int out4 = 0;

// Processed Input Signal
int left_w = 0;
int right_w = 0;

// Motor's Current
float currentValue1 = 0.0;
float currentValue2 = 0.0;
float currentValue3 = 0.0;
float currentValue4 = 0.0;
int currentLimit = 5;

//  =================================== DELAY ======================================
    unsigned long previous_LED    = 0;        // will store last time LED was updated
    unsigned long previous_Delay1 = 0;        // will store last time LED was updated
    unsigned long previous_Delay2 = 0;        // will store last time LED was updated
    unsigned long previous_Delay3 = 0;        // will store last time LED was updated
    unsigned long previous_Delay4 = 0;        // will store last time LED was updated

    unsigned long currentMillis_LED;
    unsigned long currentMillis_1;
    unsigned long currentMillis_2;
    unsigned long currentMillis_3;
    unsigned long currentMillis_4;

    const long interval = 100;                // interval at which to blink (milliseconds)
    bool StateLED = LOW;

//  ==============================================================================


//===================== setup() ========================


void setup() {
  //===== Set Digital Pin Mode =====
  // Set pinmode to read command signal from Receiver.
  pinMode(CH1, INPUT);  //channel 1
  pinMode(CH2, INPUT);  //channel 2
  pinMode(CH3, INPUT);  //channel 3
  pinMode(CH4, INPUT);  //channel 4
  pinMode(CH5, INPUT);  //channel 5
  pinMode(CH6, INPUT);  //channel 6
  // Set pinmode to read command signal from Test Switch.
  pinMode(buttonPin, INPUT);
  // Set pinmode to write command signal to Motor Driver.
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);

  pinMode(INA4, OUTPUT);
  pinMode(INB4, OUTPUT);
  pinMode(PWM4, OUTPUT);

  // Set pinmode to write command signal to LED.
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  // Assign Servo variable to a servo pin
  myServo1.attach(servo1);
  myServo2.attach(servo2);
  myServo3.attach(servo3);
  myServo4.attach(servo4);


  //===== Initialize Command =====
  // Initialize Motor Driver.
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);
  analogWrite(PWM1, 0);

  digitalWrite(INA2, LOW);
  digitalWrite(INB2, LOW);
  analogWrite(PWM2, 0);

  digitalWrite(INA3, LOW);
  digitalWrite(INB3, LOW);
  analogWrite(PWM3, 0);

  digitalWrite(INA4, LOW);
  digitalWrite(INB4, LOW);
  analogWrite(PWM4, 0);

  // Initialize Servo Motor, Set servo to Mid-point.
  myServo1.write(90);
  myServo2.write(90);
  myServo3.write(90);
  myServo4.write(90);

  // Open Serial port, Set baud rate for serial data transmission.
  Serial.begin(115200); // USB:Rx0,Tx0

  // Returns time(us)
  previousLoopTime = micros();

//Auto
  Wire.begin();
  qmc.init();

} // End SetUp


//======================= loop() ==========================


void loop() {
  input1 = pulseIn(CH2, HIGH) - 1500; //Channel 1
  input2 = pulseIn(CH1, HIGH) - 1500; //Channel 2
  input3 = pulseIn(CH3, HIGH) - 1500; //Channel 3
  input4 = pulseIn(CH4, HIGH) - 1500; //Channel 4
  input5 = pulseIn(CH5, HIGH) - 1500; //Channel 5
  input6 = pulseIn(CH6, HIGH) - 1500; //Channel 6

  input1 = Deadband(input1, 10); //Channel 1
  input2 = Deadband(input2, 10); //Channel 2
  input3 = Deadband(input3, 10); //Channel 3
  input4 = Deadband(input4, 10); //Channel 4
  input5 = Deadband(input5, 10); //Channel 5
  input6 = Deadband(input6, 10); //Channel 6

  //======================= Condition Control ==========================
  
   ////////////////////////// Auto Drive //////////////////////////////
    if( input5 < 200 ){
        qmc.read(&x,&y,&z);
        zeta = (atan2(x,z)*180/3.14);
        magnitude = (pow(x,2) + pow(z,2))/1000000;
    
        delta = zeta-lzeta;
        lzeta = zeta;
        
        //Left wheel and Right wheel 
        left_w = SpeedAuto*(1 + (Kp*(zeta)- Kd*delta));
        right_w = SpeedAuto*(1 - (Kp*(zeta)- Kd*delta));    
        //limit the maximum value of wheels
        int maxSp = 100;
        if(left_w>=maxSp)
          left_w = maxSp; 
        if(right_w>=maxSp)
          right_w = maxSp;
        
        Show();
        
        Serial.print("-->> ");
        Serial.print("  left_w: ");
        Serial.print(left_w);
        Serial.print("  right_w: ");
        Serial.println(right_w);
        
        //......................Drive Motor...........................................
        // The motor can operate if current value at each motor is not over than 5 amp.
        out1 = OutputToMotor1(left_w);
        out2 = OutputToMotor2(right_w);

        if(magnitude >= 0.5 ) 
        {
          analogWrite(PWM1,out1);
          analogWrite(PWM2,out2);
        }
        else {
          analogWrite(PWM1,0);
          analogWrite(PWM2,0);
        }
       Show_M();
       
    } else {

  
    // Set direction
        left_w  = OutputToMotor1(left_w);
        right_w = OutputToMotor2(right_w); 
        
    // Speed
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);
    }
} // End loop

//======================= Function() ==========================
void Show(){
  
    Serial.print("( ");
    Serial.print(input3);
    Serial.print(" ");
    Serial.print(input2);
    Serial.print(" ");
    Serial.print(K);
    Serial.print(" ");
    Serial.print(Zeta);
    Serial.print(" )  ");
    

}

// Motor 1
int OutputToMotor1(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    // CW
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    // CCW
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

// Motor 4
int OutputToMotor4(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    digitalWrite(INA4, LOW);
    digitalWrite(INB4, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    digitalWrite(INA4, HIGH);
    digitalWrite(INB4, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

int OutputToMotor2(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

double Deadband(double value, double limit)
{
  double temp = 0.0;
  if (value >= limit) temp = value - limit;
  else if (value <= -limit) temp = value + limit;
  else temp = 0.0;
  return temp;
}


 void Show_M(){
        Serial.print("zeta: ");
        Serial.print(zeta);
        Serial.print("  x: ");
        Serial.print(x);
        Serial.print("  y: ");
        Serial.print(y);
        Serial.print("  z: ");
        Serial.print(z);
        Serial.print("  Mag: ");
        Serial.print(magnitude);
        Serial.print("  LW: ");
        Serial.print(left_w);
        Serial.print("  RW: ");
        Serial.print(right_w);
        Serial.println(); 
 }

