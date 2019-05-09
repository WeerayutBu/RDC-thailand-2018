/*
   Set Board: Tools --> Board --> Arduino Mega2560
   Set Processor: Tools --> Processor --> ATmega2560
*/


#include <Servo.h>
#include "configuration.h"

///////////////////////////////////////////////////////////////////////////////

#define DelayServo 300
#define DelayLED 150

int degree = 10;
int temp = 90;
int pos = 150;

//Servo1 UP -> DOWN
int Up    = 30;
int Mid   = 180;
int Down  = 116;

// Arm Hold Trow
int Hold = 173;
int Trow = 120;

int indicator = 1;

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

} // End SetUp


//======================= loop() ==========================


void loop() {
  input1 = pulseIn(CH1, HIGH) - 1500; //Channel 1
  input2 = pulseIn(CH2, HIGH) - 1500; //Channel 2
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
  
   ////////////////////////  Revert Font Robot  ///////////////////////
/*
  if( input3 > 0 ){
    indicator = 0;
        
  } else if ( input3 < 0 ){
    indicator = 1; 
  }


   if( indicator == 0){
        digitalWrite(29, HIGH);
        digitalWrite(23, LOW);
   } else if( indicator == 1){
        digitalWrite(29, LOW);
        digitalWrite(23, HIGH);
        input2 = -input2;
   }
  */ 
   ////////////////////////  Control Gripper   ///////////////////////
  currentMillis_LED = millis();
  
  //Servo1
  if( input5 > 450 ){ 
    degree = Up; 
    if(input6 > 350){
      input2 = -input2;  
      if( currentMillis_LED - previous_LED >= DelayLED ){
        previous_LED = currentMillis_LED;
        //     
        digitalWrite(29, LOW);
        digitalWrite(23, StateLED);
        StateLED = !StateLED;
      }
    
    } else {
      digitalWrite(29, HIGH);
      digitalWrite(23, LOW);

    }
    
  } else if( input5 < 150 &&  input5 > -150 ){ 
      degree = Mid;
      digitalWrite(29, HIGH);
      digitalWrite(23, LOW);
       
  } else if( input5 < -450 ){  
    degree = Down;
      digitalWrite(29, HIGH);
      digitalWrite(23, LOW);
  
  }
  
  if( temp > degree){
    Arm_UP();
     
  }else if(temp < degree){
    Arm_Down(); 
  
  }
  
// Servo2
  if(input6 > 350){
    
    currentMillis_3 = millis();
    if( currentMillis_3 - previous_Delay3 >= DelayServo ){
      previous_Delay3 = currentMillis_3;
      //
      myServo2.write(Hold); 
    }
    
  } else if( input6 < -350){
    currentMillis_4 = millis();
    if( currentMillis_4 - previous_Delay4 >= DelayServo ){
      previous_Delay4 = currentMillis_4;
      //
      myServo2.write(Trow);  
    }
    
  }


   ////////////////////////  Control Wheel  ///////////////////////

    int K=1;
    input1 = input1*3/2;
    left_w = input1*(K) +(input2);
    right_w = input1*(K)-(input2);

//forward
    if (left_w>500){
      int error;
      error = left_w-500;
      left_w = 500;
      right_w -= error;
    }
    if (right_w>500){
      int error;
      error = right_w-500;
      right_w = 500;
      left_w -= error;
    }
//backward
    if (left_w<-500){
      int error;
      error = left_w+500;
      left_w = -500;
      right_w += error;
    }
    if (right_w<-500){
      int error;
      error = right_w+500;
      right_w= -500;
      left_w += error;
    }

// Set direction
    left_w  = OutputToMotor1(left_w);
    right_w = OutputToMotor4(right_w); 
    
//( Arm control )
    out2    = OutputToMotor2(input4*(-1));

// Speed
    analogWrite(PWM1, left_w);
    analogWrite(PWM4, right_w);
    analogWrite(PWM2, out2);

    Show();
    
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
    
    Serial.print("-->> ");
    Serial.print("  left_w: ");
    Serial.print(left_w);
    Serial.print("  right_w: ");
    Serial.println(right_w);
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


void Arm_UP(){
//  temp = degree;
  currentMillis_1 = millis();
  if( currentMillis_1 - previous_Delay1 >= DelayServo ){
    previous_Delay1 = currentMillis_1;
    //
    myServo1.write(degree);
  }              
}

void Arm_Down(){
 // temp = degree;
  currentMillis_2 = millis();
  if( currentMillis_2 - previous_Delay2 >= DelayServo ){
    previous_Delay2 = currentMillis_2;
    //
    myServo1.write(degree);
  }
}

