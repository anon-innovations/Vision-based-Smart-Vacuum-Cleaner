#include <SoftwareSerial.h>
// Reference the I2C Library
#include <Wire.h>
// Reference the HMC5883L Compass Library
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

int error = 0;

SoftwareSerial BT(10, 11); 
// creates a "virtual" serial port/UART
// connect BT module TX to D10
// connect BT module RX to D11
// connect BT Vcc to 5V, GND to GND
//Distance away
int distance;

//Sets the duration each keystroke captures the motors.
int keyDuration = 100;

int iComp;

//we define the pins that set the motors' direction
int pwm_a = 5;  //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 6;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int motor1a = 2; 
int motor1b = 3;
int motor2a = 4;
int motor2b = 7;

int lowspeed = 100;
int highspeed = 100;

void setup()  
{
  Serial.begin(9600);
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  //Wire.begin(); // Start the I2C interface.

 /* Serial.println("Constructing new HMC5883L");
 // compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  //error = compass.SetScale(1.3); // Set the scale of the compass
  //error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous*/
   
  // we define the motor pins as outputs
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(motor1a, OUTPUT);
  pinMode(motor1b, OUTPUT);
  pinMode(motor2a, OUTPUT);
  pinMode(motor2b, OUTPUT);

  analogWrite(pwm_a, 0);        
  //set both motors to run at (100/255 = 39)% duty cycle (slow)  
  analogWrite(pwm_b, 0);
  
  BT.begin(9600);// set the data rate for the SoftwareSerial port
  Serial.begin(9600);
 
}

char a; // stores incoming character from other device

void loop() 
{
  Serial.begin(9600);

  sensors_event_t event; 
  mag.getEvent(&event);
     
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);  
    
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2ï¿½ 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -2.3;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;  
  
  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);

  //This throttles how much data is sent to Python code.  
  //Basically, it updates every second (10 microsecond delay X 100 iComps)
  if (iComp >= 30){
    
    int adjHeading = 0;    
    //The "floor" part makes the float into an integer, rounds it up.
    headingDegrees = floor(headingDegrees);
       
    if (headingDegrees >= 280){
        adjHeading = map(headingDegrees, 280, 360, 0, 79);
    }
    else if (headingDegrees <= 279) {
        adjHeading = map(headingDegrees, 0, 279, 80, 360);
    }
    //Serial.println(adjHeading);
    BT.println(adjHeading);
    Serial.println(adjHeading);
    iComp=0;
  }
  iComp++;

delay(10); 
  
if (BT.available())
    {
    a=(BT.read());//it will be read and
    Serial.println(a);
    if (a=='1')//if a=1 (the signal from the 'up' button from the bluetooth app) it will move forward
    {
//set both motors to run at 100% duty cycle (fast)
      analogWrite(pwm_a, highspeed);      
      analogWrite(pwm_b, highspeed);
     
      digitalWrite(motor1a, HIGH);
      digitalWrite(motor1b, LOW);
      digitalWrite(motor2a, HIGH);
      digitalWrite(motor2b, LOW);   

delay(keyDuration);
    }
    if (a=='2')//it will move backwards
    {
//set both motors to run at 100% duty cycle (fast)
      analogWrite(pwm_a, highspeed);      
      analogWrite(pwm_b, highspeed);
      
     digitalWrite(motor1a, LOW);
     digitalWrite(motor1b, HIGH);
     digitalWrite(motor2a, LOW);
     digitalWrite(motor2b, HIGH);
delay(keyDuration);    
    }    
    if (a=='3')// it will move left
    {
//Left
      analogWrite(pwm_a, lowspeed);      
      analogWrite(pwm_b, lowspeed);
      
     digitalWrite(motor1a, HIGH);
     //digitalWrite(motor1b, LOW);
     //digitalWrite(motor2a, HIGH);
     //digitalWrite(motor2b, HIGH); 
      
delay(keyDuration); 
    }    
    if (a=='4')//it will move right
    {
//Right
      analogWrite(pwm_a, lowspeed);      
      analogWrite(pwm_b, lowspeed);
      
     //digitalWrite(motor1a, LOW);
     //digitalWrite(motor1b, HIGH);
     digitalWrite(motor2a, HIGH);
     //digitalWrite(motor2b, HIGH);
       
delay(keyDuration);   
    }    
    if (a=='0')//it will not move
    {
//set both motors to run at 100% duty cycle (fast)
      analogWrite(pwm_a, 0);      
      analogWrite(pwm_b, 0);
  
     digitalWrite(motor1a, LOW);
     digitalWrite(motor1b, LOW);
     digitalWrite(motor2a, LOW);
     digitalWrite(motor2b, LOW);
     
  delay(keyDuration);
    }    
  }
}
