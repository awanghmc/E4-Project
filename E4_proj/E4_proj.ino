/*

Source Code: RedBearLabs, SimpleController Source
Modified by: Aaron Wang
HMC E4 Section 1, Team 3
Fall 2014
Made for Praxis Biosciences, Test Tube Rocker/Product Display

Instructions:
After bluetooth connection with the RedBearLabs smartphone app (SimpleControls) , first engage the PWM to set 
motorspeed, or the rotational speed. It is important to set a rotational speed BEFORE enabling shaking mode, otherwise
the shaking mode will proceed without rotation. Shaking mode is enabled by switching ON/OFF with the digital out 
switch (Pin 2). The servo slider will change the angle of the servo ONLY when shaking mode is disabled. 

Product Display Mode (order not important):
-Set PWM (rotation speed)
-Set Servo (platform angle)

Shaking Mode (order is important):
1. Set PWM (rotation speed)
2. Turn on Digital Out pin
3. To turn off, turn off Digital Out pin then set PWM to 0

*/

#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <Servo.h>
#include <Stepper.h>

#define DIGITAL_OUT_PIN    2
#define DIGITAL_IN_PIN     A4
#define PWM_PIN            3
#define SERVO_PIN          5
#define ANALOG_IN_PIN      A5

const int stepsPerRev = 200; //unipolar stepper motor
Stepper myStepper(stepsPerRev,3,4,6,7); //initialize motor on pins 3,4,6,7. Pin 5 is reserved for Servo on iOS app.
int motorSpeed=0; //set initial motor speed at 0
Servo myservo;
int pos = 0;
boolean sweep=false; //shaking mode, off initially

void setup()
{
  // Default pins set to 9 and 8 for REQN and RDYN
  // Set your REQN and RDYN here before ble_begin() if you need
  //ble_set_pins(3, 2);
  
  // Set your BLE Shield name here, max. length 10
  //ble_set_name("My Name");
  
  // Init. and start BLE library.
  ble_begin();
  
  // Enable serial debug
  Serial.begin(9600);
  
  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  pinMode(DIGITAL_IN_PIN, INPUT);
  
  // Default to internally pull high, change it if you need
  digitalWrite(DIGITAL_IN_PIN, HIGH);
  //digitalWrite(DIGITAL_IN_PIN, LOW);
  
  myservo.attach(SERVO_PIN);
}

void loop()
{
  static boolean analog_enabled = false;
  static byte old_state = LOW;
  
  while(ble_available())
  {
    // read out command and data
    byte data0 = ble_read();
    byte data1 = ble_read();
    byte data2 = ble_read();
    
    if (data0 == 0x01)  // Command is to control digital out pin
    {
      if (data1 == 0x01) //if pin 2 turned on, shaking mode is on
        sweep = true;    
      else
        sweep = false;
    }
    
    else if (data0 == 0xA0) // Command is to enable analog in reading
    {
      if (data1 == 0x01)
        analog_enabled = true;
      else
        analog_enabled = false;
    }
    else if (data0 == 0x02) // Command is to control PWM pin
    //data1 = analog in
    {
      //Serial.println(data1); 
      int sensorReading = data1; //gets the analog reading from PWM input
      // map it to a range from 0 to 100:
      motorSpeed = map(sensorReading, 0, 255, 0, 100);
      //Serial.println(motorSpeed);
      // set the motor speed:
      
    }
    else if (data0 == 0x03)  // Command is to control Servo pin/ Servo actuation
    {
      myservo.write(data1);
    }
    else if (data0 == 0x04)
    {
      analog_enabled = false;
      myservo.write(0);
      analogWrite(PWM_PIN, 0);
      digitalWrite(DIGITAL_OUT_PIN, LOW);
    }
  }
  
  
  if(sweep==true){ 
       for(pos = 0; pos <= 100; pos += 1) // goes from 0 degrees to 100 degrees 
       {                                  // in steps of 1 degree 
          myservo.write(pos);             // Stepper also steps during each degree increment
          myStepper.setSpeed(motorSpeed); // NOTE: the motorSpeed is supposed to be set BEFORE shaking mode is enabled
        // step 1/100 of a revolution:
          myStepper.step(stepsPerRev/100);   
         
       } 
       for(pos = 100; pos>=0; pos-=1)     // goes from 100 degrees to 0 degrees 
       {                                
         myservo.write(pos);              // tell servo to go to position in variable 'pos' 
         myStepper.setSpeed(motorSpeed);
        // step 1/100 of a revolution:
         myStepper.step(stepsPerRev/100);    
         
       }  
  }
   
  if (motorSpeed >0) { //if shaking mode is not on, then only rotation occurs via stepper
         Serial.println("running");
        myStepper.setSpeed(motorSpeed);
        // step 1/100 of a revolution:
        myStepper.step(stepsPerRev/100);}
        
        
        
  if (analog_enabled)  // if analog reading enabled
  {
    // Read and send out
    uint16_t value = analogRead(ANALOG_IN_PIN); 
    ble_write(0x0B);
    ble_write(value >> 8);
    ble_write(value);
  }
  
  // If digital in changes, report the state
  if (digitalRead(DIGITAL_IN_PIN) != old_state)
  {
    old_state = digitalRead(DIGITAL_IN_PIN);
    
    if (digitalRead(DIGITAL_IN_PIN) == HIGH)
    {
      ble_write(0x0A);
      ble_write(0x01);
      ble_write(0x00);    
    }
    else
    {
      ble_write(0x0A);
      ble_write(0x00);
      ble_write(0x00);
    }
  }
  
  if (!ble_connected())
  {
    analog_enabled = false;
    digitalWrite(DIGITAL_OUT_PIN, LOW);
  }
  
  // Allow BLE Shield to send/receive data
  ble_do_events();  
}

