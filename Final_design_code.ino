#include <TwiMap.h>

#include "MLX90615.h"
#include <I2cMaster.h>

#define SDA_PIN 5   //define the SDA digital pin
#define SCL_PIN 4   //define the SCL digital pin

SoftI2cMaster i2c(SDA_PIN, SCL_PIN);
MLX90615 mlx90615(DEVICE_ADDR, &i2c);

const int pingPin = 12;
const int trigPin = 7;
const int echoPin = 8;

int r0 = 0;      //pin(s0)
int r1 = 1;      //pin (s1)
int r2 = 2;      //pin (s2)

int count = 0;   
int count2 = 0;
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
    Serial.write(data[control]);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(0, OUTPUT);    // s0
  pinMode(1, OUTPUT);    // s1
  pinMode(2, OUTPUT);    // s2

  pinMode( 9 , OUTPUT);   //vibration motors 1-4
  pinMode( 10 , OUTPUT);   //motors go right to left from seated position
  pinMode( 11 , OUTPUT);
  pinMode( 13 , OUTPUT);
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
    for(count=0; count<=7; count++) 
    {
      // selecting algorithms  
      r0 = bitRead(count,0);        
      r1 = bitRead(count,1);       
      r2 = bitRead(count,2);      
      count2 = count+8;
      digitalWrite(0, r0); 
      digitalWrite(1, r1);
      digitalWrite(2, r2);
    
      pressureArray[count] = analogRead(0); //analog output at A0
      pressureArray[count2] = analogRead(1);//analog output at A1
  
      if(pressureArray[count] > 15)
        pressCount++;                    //pressure has been sensed; counts how many "fsr" were pressed
    }

    delay(1000);
    
    //if at least 4 pressure sensors are pressed then person is seated
    if(ambientTemp < objectTemp && pressCount > 3)
    {
      for(count=0; count<=7; count++) 
      {
        //selecting algorithms
        r0 = bitRead(count,0);         
        r1 = bitRead(count,1);       
        r2 = bitRead(count,2);      
        count2 = count+8;
        digitalWrite(0, r0); 
        digitalWrite(1, r1);
        digitalWrite(2, r2);
  
        pressureArray[count] = analogRead(0); //analog output at A0
        pressureArray[count2] = analogRead(1);//analog output at A1

        if(count == 0 || count == 1 || count == 2 || count ==  3)
          outerRightSum += pressureArray[count];

        if(count2 == 12 || count2 == 13 || count2 == 14 || count2 ==  15)
          outerLeftSum += pressureArray[count2];
      }
      
      //Calibration; this is only done once
      while(outerRightSum < outerLeftSum*(.9) || outerLeftSum < outerRightSum*(.9) || outerRightSum == 0 || outerLeftSum == 0)
      {
        Serial.println("Sit properly...trying to calibrate");
        outerRightSum = 0;
        outerLeftSum = 0;
        pressCount = 0;

        //vibrates the chair 3 times to notify the user to sit properly
        for(count=0; count<3; count++)
        {
          analogWrite( 9 , 153 );  // 60% duty cycle
          analogWrite( 10 , 153 );
          analogWrite( 11 , 153 );
          analogWrite( 13 , 153 );
          delay(1000);            // on time
          analogWrite( 9 , 0 );
          analogWrite( 10 , 0 );    // 0% duty cycle (off)
          analogWrite( 11 , 0 );
          analogWrite( 13 , 0 );
          delay(1000);
        }
        
        for(count=0; count<=7; count++) 
        {
          //selecting algorithms
          r0 = bitRead(count,0);         
          r1 = bitRead(count,1);       
          r2 = bitRead(count,2);      
          count2 = count+8;
          digitalWrite(0, r0); 
          digitalWrite(1, r1);
          digitalWrite(2, r2);
    
          pressureArray[count] = analogRead(0); //analog output at A0
          pressureArray[count2] = analogRead(1);//analog output at A1

          if(pressureArray[count] > 15)
            pressCount++; 
  
          if(count == 0 || count == 1 || count == 2 || count ==  3)
            outerRightSum += pressureArray[count];

          if(count2 == 12 || count2 == 13 || count2 == 14 || count2 ==  15)
            outerLeftSum += pressureArray[count2];

        }
        
        //If no one is there break out
        if(pressCount < 2)
          break;
      }

      if(pressCount < 2)
        Serial.println("Not Calibrated");
      else
        Serial.println("System calibrated");

      
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
        
        for(count=0; count<=7; count++) 
        {
          //selecting algorithms
          r0 = bitRead(count,0);         
          r1 = bitRead(count,1);       
          r2 = bitRead(count,2);      
          count2 = count+8;
          digitalWrite(0, r0); 
          digitalWrite(1, r1);
          digitalWrite(2, r2);

          pressureArray[count] = analogRead(0); //analog output at A0
          pressureArray[count2] = analogRead(1);//analog output at A1
  
          leftSum += pressureArray[count2]; //sum pressure sensors 9-16
          rightSum += pressureArray[count];
        }

        rightSum = rightSum - 300;
        
        frontSum = pressureArray[0]+pressureArray[1]+pressureArray[4]+pressureArray[5]+pressureArray[8]+pressureArray[9]+pressureArray[12]+pressureArray[13];
        backSum = pressureArray[2]+pressureArray[3]+pressureArray[6]+pressureArray[7]+pressureArray[10]+pressureArray[11]+pressureArray[14]+pressureArray[15];

        delay(3000);
  
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
  
        cm2 = microsecondsToCentimeters(duration2) - 18;

       /*Serial.print("cm1: ");
       Serial.println(cm);
       Serial.println(cm2);*/
       if((leftSum + rightSum) < 400)
         {
          Serial.println("User stood up");
          break;
         }
        else if(frontSum > backSum)
        {
          Serial.println("Sitting too far up...scoot back");
        }
        else if(leftSum < rightSum*(.88))              
        {
          Serial.println("Favoring right side");  
          analogWrite( 9 , 153 );  // 60% duty cycle
          analogWrite( 10 , 153 );
          delay(2000);            // on time
          analogWrite( 9 , 0 );
          analogWrite( 10 , 0 );    // 0% duty cycle (off)
          delay(1000);
        }
        else if(rightSum < leftSum*(.88))
        {
          Serial.println("Favoring left side");

          for(count=0; count<2; count++)
          {
            analogWrite( 11 , 153 );
            analogWrite( 13 , 153 );
            delay(2000);            // on time
            analogWrite( 11 , 0 );
            analogWrite( 13 , 0 );
            delay(1000);
          }
        }
        else if(cm > 10 && frontSum > backSum)//might need to edit
        {
          Serial.println("Sitting too forward...scoot back");
  
          //vibrates front motors 3 times
          for(count=0; count<3; count++)
          {
            analogWrite( 9 , 153 );  // 60% duty cycle
            analogWrite( 11 , 153 );
            delay(1000);            // on time
            analogWrite( 9 , 0 );
            analogWrite( 11 , 0 );    // 0% duty cycle (off)
            delay(1000);
          }
        }
        else if(cm2 > cm + 15 && cm2 < 40)
          Serial.println("Leaning forward, sit up straight");
        else if(cm > cm2 + 3)
          Serial.println("Leaning back, sit up straight");

         
         
      }//ends infinite loop
    }//closes "if" statement that is triggered by pressCount
   
  }//this closes the first "if" statment...the temp sensor indicatior
}



long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
