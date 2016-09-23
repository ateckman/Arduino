//DO NOT EDIT CODE

#include <Servo.h>

// RC dont change
#define CH1_int 0    // Channel 1 interrupt # 
#define CH1_pin 2     // Respective channel Hardware interrupt pin number

#define CH2_int 1     // Channel 2 interrupt # 
#define CH2_pin 3     // Respective channel Hardware interrupt pin number

#define valid_pulse_limit 2000 // [uS] Valid output high pulse time limit for RC controller
#define max_high_time 2300 // [uS] Maximum expected high time
#define min_high_time 500 // [uS] Minimum expected high time
//
//DO NOT EDIT CODE ABOVE

//THIS IS THE ONLY CODE YOUR EDIT*******************************************************
//**************************************************************************************
int b = 20; //This is distance in feet forward
int a = 2; 
//**************************************************************************************
//**************************************************************************************
//DO NOT EDIT CODE
//DO NOT EDIT CODE
//    a = Speed
// 1 = Fast forward
// 2 = medium forward
// 3 = slow forward **time fix
// 4 == slow back **time fix
// 5 == medium back
// 6 == fast back

int power_mapping [] = { 0, 160, 135, 110, 70, 50, 30 }; //Speed Settings
int power_mappingReversed [] = { 0, 30, 50, 70, 110, 135, 160 }; //Speed Settings
float timer_mapping [] = { 0, 131, 160, 1245, 1131, 150, 137 }; //Speed in Feet Per Second
float timer_mappingReversed [] = { 0, 137, 150, 1131, 1245, 160, 131 }; //Speed in Feet Per Second
//160(0546) 135(1139) 110(4150) 70(3770) 50(1298) 30(0690)

int channel1 = 2; // defines the channels that are connected
int channel2 = 3;// to pins 9 and 10 of arduino respectively
int analogOutputF ; //Used later to
int analogOutputB ; // store values
int Channel1 ; // Used later to 
int Channel2 ; // store values
int switchingServo = 0;
volatile unsigned long CH1_t=0, CH1_delta=0, CH2_t=0, CH2_delta=0 ;
float CH1,CH2;
unsigned long t=0;
//------- VARIABLES (will change)

Servo ST1;
Servo ST2;  
int servoInt = 0;          //    will be changed to negative value for movement in the other direction
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long previousServoMillis = 0; // the time when the servo was last moved


int count;   // in declarations
int ch1_previousDelta;
int ch2_previousDelta;
boolean transmitterOff = true;


//========
void CH1_int_ISR()
{
  if ((micros()-CH1_t) < valid_pulse_limit){
    CH1_delta = micros()-CH1_t;
  }
  CH1_t = micros();
}

void CH2_int_ISR()
{
  if ((micros()-CH2_t) < valid_pulse_limit){
    CH2_delta = micros()-CH2_t;
  }
  CH2_t = micros();

}

// Call able function



//========
void setup() {

   pinMode (channel1, INPUT);// initialises the channels
   pinMode (channel2, INPUT);// as inputs
   Serial.begin(115200);
   ST1.attach(8);
   ST2.attach(7);
  Serial.print("Channel 1 connected to pin number....\t");
  Serial.println(CH1_pin);
  Serial.print("Channel 2 connected to pin number....\t");
  Serial.println(CH2_pin);
  
  pinMode(CH1_pin, INPUT);
  pinMode(CH2_pin, INPUT);

  attachInterrupt(CH1_int, CH1_int_ISR, CHANGE);
  attachInterrupt(CH2_int, CH2_int_ISR, CHANGE);


}


//=======

void loop() {
  
  
  
if(CH1_delta == ch1_previousDelta && CH2_delta == ch2_previousDelta)
{
  count++;
 // Serial.println(count);
}
else
{
ch1_previousDelta = CH1_delta;
ch2_previousDelta = CH2_delta;
transmitterOff = false;
count=0;
}

if(count>5000)
{
transmitterOff = true;
}



currentMillis = millis();
 // Serial.println(CH1_delta);
 // Serial.println(CH2_delta);

if (transmitterOff){

 if (currentMillis - previousServoMillis >= servoInt) {
      previousServoMillis = currentMillis;

    if (switchingServo == 0){
      switchingServo = 1;
      ST1.write(power_mapping [a]);
      servoInt = (b*timer_mapping [a]);
      Serial.println(servoInt);

    }else if (switchingServo == 1){
      switchingServo = 2;
      //Back Wheel
      ST1.write(90);
      //Front Wheel
     // ST2.write(180); EDITED OUT FRPONT WHEEL MOTION
      servoInt = (1000);
      Serial.println(servoInt);

    }else if (switchingServo == 2){
      switchingServo = 3;
      ST1.write(power_mappingReversed [a]);
      servoInt = (b*timer_mappingReversed [a]);
      Serial.println(servoInt); 
      
    }else if (switchingServo == 3){
      switchingServo = 0;
      //Back Wheel
      ST1.write(90);
      //Front Wheel
      //ST2.write(0); EDITED OUT FRPONT WHEEL MOTION
      servoInt = (1000);
      Serial.println(servoInt);

    }}
  }else{
       
   previousServoMillis = 0; 
   servoInt = 0; //delay for program
   switchingServo = 0;
   
   
     //BACK MOTOR
   analogOutputF = map(CH2_delta, 1060, 2000, 0, 180); 
    ST1.write(analogOutputF);
    // Serial.println (analogOutputF);
      //STEERING  
   analogOutputB = map(CH1_delta, 1260, 1660, 0, 180); 
   
             Serial.println(CH1_delta);

   if (CH1_delta > 1550 && CH1_delta < 1700){
     ST2.write(180);

    //  Serial.println (180);

   }else if (CH1_delta < 1475 && CH1_delta < 1300){
     ST2.write(0);
    //  Serial.println (0);

   }else if (CH1_delta > 1475 && CH1_delta < 1525){
     ST2.write(90);
    //  Serial.println (110);
   
    }
  }
}



//=====END

