/*************************************************************
  File:      TestPrograms.ino
  Contents:  This program demonstrates simple control and 
  capabilities of the robot. Particularly, it is designed
  to act in response to a certain keystroke.
1 . turn the motors on ("y") and off
2. get a character from the keyboard
3. print a value to the serial monitor
4. read the light sensor 
5. test the IR bumpers
  Notes:     Target: Arduino Leonardo
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-01-09 LAK and ZDR  program created
 ************************************************************/
 
/*---------------Includes-----------------------------------*/

/*---------------Module Defines-----------------------------*/
#define LIGHT_THRESHOLD    350 // smaller at night
#define FENCE_THRESHOLD    700
#define ONE_SEC            1000
#define TIME_INTERVAL      ONE_SEC

void setup() {
  
  Serial.begin(9600);
}

void loop() {
  if (TestForKey()) RespToKey();

}

unsigned char TestForKey(void) {
unsigned char KeyEventOccurred;
  
  KeyEventOccurred = Serial.available();
  return KeyEventOccurred;
}

void RespToKey(void) {
  unsigned char theKey;
 
  theKey = Serial.read();
  
  Serial.write(theKey);
  Serial.print(", ASCII=");
  Serial.println(theKey,HEX);
  
  sparki.println(theKey);
  sparki.updateLCD();
 
   if (theKey == 'o') {
   motorOn();
  }
   if (theKey=='n') {
   motorOff();
  }
   if (theKey=='v') {
   Serial.println(12);
  }
   if (theKey == 'l'){
   Serial.println("Light =" );
   Serial.println(LightLevel());
   }

   if(theKey == 'I'){
    readIRBumpers();
   }
  
}

void motorOn(void) {
  sparki.println(" ON");
  sparki.updateLCD();
  LeftMtrSpeed(0);
  RightMtrSpeed(10);
}

void motorOff(void) {
  sparki.println(" OFF");
  sparki.updateLCD();
  LeftMtrSpeed(0);
  RightMtrSpeed(0);
}

void readIRBumpers(void){

Serial.println("Edge Sensor Reading (Right) =");
Serial.println(  sparki.edgeRight());;
Serial.println("Line Sensor Reading (Right) =");
Serial.println(  sparki.lineRight());
Serial.println("Line Sensor Reading (Center) =");
Serial.println(  sparki.lineCenter());
Serial.println("Line Sensor Reading (Left) =");
Serial.println(  sparki.lineLeft());  
Serial.println("Edge Sensor Reading (Left) =");
Serial.println(  sparki.edgeLeft());
   }

