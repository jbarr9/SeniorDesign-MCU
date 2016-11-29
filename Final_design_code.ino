#include <TwiMap.h>

#include "MLX90615.h"
#include <I2cMaster.h>

#define SDA_PIN 2   //define the SDA digital pin
#define SCL_PIN 3   //define the SCL digital pin

SoftI2cMaster i2c(SDA_PIN, SCL_PIN);
MLX90615 mlx90615(DEVICE_ADDR, &i2c);

const int pingPin = 9;
const int trigPin = 10;
const int echoPin = 11;
const int motor1 = 5; //9
const int motor2 = 6; //10
const int motor3 = 12; //11
const int motor4 = 13; //13
//Select lines
const int sel0 = 17; //make sure to use Analog pins
const int sel1 = 16;
const int sel2 = 15;
const int sel3 = 14;
const int muxEn = 18;
const int analogOn = 255;
const int analogOff = 0;


int r0 = 3;      //pin (s0) select lines
int r1 = 2;      //pin (s1) select lines
int r2 = 1;      //pin (s2) select lines
int r4 = 0;      //pin (s3) select lines
const int muxRead = 5; //pin reads in mux value

int count = 0;   
int pressureArray[16];  //stores pressure values into array; used for organizational purposes

float ambientTemp = 0;
float objectTemp = 0;

long duration, in, cm;
long duration2, in2, cm2;

int pressCount = 0;
int outerRightSum = 0;
int outerLeftSum = 0;
int leftSum = 0;
int rightSum = 0;
int frontSum = 0;
int backSum = 0;

//////////////////////////////////////////////////////
// Sends data to the wifi module, data is stored in
// global variables, so no arguments necessary.
//////////////////////////////////////////////////////
void sendWifi(void)
{
  byte data[42];
  int control = 0, dataCont = 0;

  //splits the first duration into bytes
  for(control = 0; control < 3; control++)
  {
    data[dataCont] = (duration >> 24 - 8 * control) & 0x000000FF;
    dataCont++;
  }

  //splits the second duration into bytes
  for(control = 0; control < 3; control++)
  {
    data[dataCont] = (duration2 >> 24 - 8 * control) & 0x000000FF;
    dataCont++;
  }

  //Get bytes of pressure array, each reading at once
  for(control = 0; control < 16; control++)
  {
    data[dataCont] = (pressureArray[control] >> 8) & 0x00FF;
    dataCont++;
    data[dataCont] = (pressureArray[control]) & 0x00FF;
    dataCont++;
  }
  int temp = (int)(objectTemp * 1000);
  data[dataCont] = (temp >> 8) & 0x00FF;
  dataCont++;
  data[dataCont] = (temp) & 0x00FF;

  Serial.write(68);
  for(control = 0; control < 42; control++)
  {
   // Serial.write(data[control]);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(sel0, OUTPUT);    // s0
  pinMode(sel1, OUTPUT);    // s1
  pinMode(sel2, OUTPUT);    // s2
  pinMode(sel3, OUTPUT);    // s3
  pinMode(muxEn, OUTPUT);
  digitalWrite(muxEn, HIGH); //always enabled

  pinMode( motor1 , OUTPUT);   //vibration motors 1-4
  pinMode( motor2 , OUTPUT);   //motors go right to left from seated position
  pinMode( motor3 , OUTPUT);
  pinMode( motor4 , OUTPUT);
}

void loop () {

  outerRightSum = 0;
  outerLeftSum = 0;
  leftSum = 0;
  rightSum = 0;
  frontSum = 0;
  backSum = 0;
  pressCount = 0;

  //Storing temperature sensor readings
  objectTemp = mlx90615.getTemperature(MLX90615_OBJECT_TEMPERATURE) + 5;
  ambientTemp = mlx90615.getTemperature(MLX90615_AMBIENT_TEMPERATURE);

  if(ambientTemp < objectTemp)
  {
    
    //Detecting if anything is on the chair
    for(count=0; count<=15; count++) 
    {
      // selecting algorithms  
      r0 = bitRead(count,0);        
      r1 = bitRead(count,1);       
      r2 = bitRead(count,2);
      r3 = bitRead(count,3);     
      
      digitalWrite(sel0, r0); 
      digitalWrite(sel1, r1);
      digitalWrite(sel2, r2);
      digitalWrite(sel3, r3);
    
      pressureArray[count] = analogRead(muxRead); //analog output at A0
  
      if(pressureArray[count] > 15)
        pressCount++;                    //pressure has been sensed; counts how many "fsr" were pressed
    }

    delay(1000);
    
    //if at least 4 pressure sensors are pressed then person is seated
    if(ambientTemp < objectTemp && pressCount > 3)
    {
      for(count=0; count<=15; count++) 
      {
        //selecting algorithms
        r0 = bitRead(count,0);         
        r1 = bitRead(count,1);       
        r2 = bitRead(count,2);      
        
        digitalWrite(sel0, r0); 
        digitalWrite(sel1, r1);
        digitalWrite(sel2, r2);
        digitalWrite(sel3, r3);
  
        pressureArray[count] = analogRead(muxRead); //analog output at A0

        if(count == 0 || count == 1 || count == 2 || count == 3)
          outerRightSum += pressureArray[count];

        if(count == 12 || count == 13 || count == 14 || count == 15)
          outerLeftSum += pressureArray[count];
      }
      
      //Calibration; this is only done once initially
      while(outerRightSum < outerLeftSum*(.9) || outerLeftSum < outerRightSum*(.9))
      {
        Serial.println("Sit properly...trying to calibrate");
        outerRightSum = 0;
        outerLeftSum = 0;
        pressCount = 0;

        //vibrates the chair 3 times to notify the user to sit properly
        for(count=0; count<3; count++)
        {
          analogWrite( motor1 , 153 );  // 60% duty cycle
          analogWrite( motor2 , 153 );
          analogWrite( motor3 , 153 );
          analogWrite( motor4 , 153 );
          delay(1000);            // on time
          analogWrite( motor1 , 0 );
          analogWrite( motor2 , 0 );    // 0% duty cycle (off)
          analogWrite( motor3 , 0 );
          analogWrite( motor4 , 0 );
          delay(1000);
        }
        
        for(count=0; count<=15; count++) 
        {
          //selecting algorithms
          r0 = bitRead(count,0);         
          r1 = bitRead(count,1);       
          r2 = bitRead(count,2);      
          r3 = bitRead(count,3);
          
          digitalWrite(sel0, r0); 
          digitalWrite(sel1, r1);
          digitalWrite(sel2, r2);
          digitalWrite(sel3, r3);
    
          pressureArray[count] = analogRead(muxRead); //analog output at A0

          if(pressureArray[count] > 15)
            pressCount++; 
  
          if(count == 0 || count == 1 || count == 2 || count ==  3)
            outerRightSum += pressureArray[count];

          if(count == 12 || count == 13 || count == 14 || count ==  15)
            outerLeftSum += pressureArray[count];

        }
        
        //If no one is there break out
        if(pressCount < 2)
          break;
      }

      if(pressCount < 2)
        Serial.println("Not calibrated");
      else
      {
        Serial.println("System calibrated");
        analogWrite( motor1 , 153 );  // 60% duty cycle
        analogWrite( motor2 , 153 );
        analogWrite( motor3 , 153 );
        analogWrite( motor4 , 153 );
        delay(1000);            // on time
        analogWrite( motor1 , 0 );
        analogWrite( motor2 , 0 );    // 0% duty cycle (off)
        analogWrite( motor3 , 0 );
        analogWrite( motor4 , 0 );
        delay(1000);
      }

      
      //Infinite Loop
      for(;;)
      {
        if(pressCount < 2)
        {
          break;
        }
          
        leftSum = 0;
        rightSum = 0;
        frontSum = 0;
        backSum = 0;
        
        for(count=0; count<=15; count++) 
        {
          //selecting algorithms
          r0 = bitRead(count,0);         
          r1 = bitRead(count,1);       
          r2 = bitRead(count,2);
          r3 = bitRead(count,3);
          
          digitalWrite(sel0, r0); 
          digitalWrite(sel1, r1);
          digitalWrite(sel2, r2);
          digitalWrite(sel3, r3);

          pressureArray[count] = analogRead(muxRead); //analog output at A0
  
          if(count > 7)
            leftSum += pressureArray[count]; //sum pressure sensors 9-16
          else
            rightSum += pressureArray[count];
        }

        rightSum = rightSum - 300; //Evens out the pressure sensors
        
        frontSum = pressureArray[0]+pressureArray[1]+pressureArray[4]+pressureArray[5]+pressureArray[8]+pressureArray[9]+pressureArray[12]+pressureArray[13];
        backSum = pressureArray[2]+pressureArray[3]+pressureArray[6]+pressureArray[7]+pressureArray[10]+pressureArray[11]+pressureArray[14]+pressureArray[15];

        delay(2000);
  
        //Set up for 3 pin distance sensor
        pinMode(pingPin, OUTPUT);
        digitalWrite(pingPin, LOW);
        delayMicroseconds(2);
        digitalWrite(pingPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pingPin, LOW);
      
        pinMode(pingPin, INPUT);
        duration = pulseIn(pingPin, HIGH);
  
        cm = microsecondsToCentimeters(duration);
  
        //Setup for 4 pin distance sensor
        pinMode(trigPin, OUTPUT);
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
  
        pinMode(echoPin, INPUT);
        duration2 = pulseIn(echoPin, HIGH);
  
        cm2 = microsecondsToCentimeters(duration2) - 18; //Sets both distance sensors at equilibrium
        
       if((leftSum + rightSum) < 400)
         {
          Serial.println("User stood up");
          break;
         }
        else if(leftSum < rightSum*(.88))              
        {
          Serial.println("Favoring right side");  
          analogWrite( motor1 , 153 );  // 60% duty cycle
          analogWrite( motor2 , 153 );
          delay(2000);            // on time
          analogWrite( mootr1 , 0 );
          analogWrite( motor2 , 0 );    // 0% duty cycle (off)
          delay(1000);
        }
        else if(rightSum < leftSum*(.88))
        {
          Serial.println("Favoring left side");

          for(count=0; count<2; count++)
          {
            analogWrite( motor3 , 153 );
            analogWrite( motor4 , 153 );
            delay(1000);            // on time
            analogWrite( motor3 , 0 );
            analogWrite( motor4 , 0 );
            delay(1000);
          }
        }
        else if(cm > 10 && frontSum > backSum)//might need to edit
        {
          Serial.println("Sitting too far up...scoot back");
  
          //vibrates front motors 3 times
          for(count=0; count<3; count++)
          {
            analogWrite( motor1 , 153 );  // 60% duty cycle
            analogWrite( motor3 , 153 );
            delay(1000);            // on time
            analogWrite( motor1 , 0 );
            analogWrite( motor3 , 0 );    // 0% duty cycle (off)
            delay(1000);
          }
        }
        else if(cm2 > cm + 15 && cm2 < 40)
        {
          Serial.println("Leaning forward, sit up straight");
          for(count=0; count<2; count++)
          {
            analogWrite( motor1 , 153 );  // 60% duty cycle
            analogWrite( motor3 , 153 );
            delay(1000);            // on time
            analogWrite( motor1 , 0 );
            analogWrite( motor3 , 0 );    // 0% duty cycle (off)
            delay(1000);

            analogWrite( motor2 , 153 );  // 60% duty cycle
            analogWrite( motor4 , 153 );
            delay(1000);            // on time
            analogWrite( motor2 , 0 );
            analogWrite( motor4 , 0 );    // 0% duty cycle (off)
            delay(1000);
          }
        }
        else if(cm > cm2 + 3)
        {
          Serial.println("Leaning back, sit up straight");
          for(count=0; count<2; count++)
          {
            analogWrite( motor1 , 153 );  // 60% duty cycle
            analogWrite( motor3 , 153 );
            delay(3000);            // on time
            analogWrite( motor1 , 0 );
            analogWrite( motor3 , 0 );    // 0% duty cycle (off)
            delay(1000);
          }
        }

        sendWifi();
         
      }//ends infinite loop
      
    }//closes "if" statement that is triggered by pressCount
   
  }//this closes the first "if" statment...the temp sensor indicatior
  
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
