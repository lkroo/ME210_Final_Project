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
/*------ ROBOT CONFIGURATION FILE --------*/

// ANALOG SENSORS
#define CLIP_SENSE        A0 // clip sensor
#define LINE_BACK_SENSE   A1 // back line sensor
#define LINE_RIGHT_SENSE  A2 // right line sensor
#define LINE_CENTER_SENSE A3 // center line sensor
#define LINE_LEFT_SENSE   A4 // left line sensor
#define FLYWHEEL_SENSE    A5 // Flywheel speed sensor
// DIGITAL PINS
#define SOLENOID       2 // 
#define FLYWHEEL       3 // 
#define BEACON1        5 //
#define BEACON2        6 // 
#define SERVO_PAN      9 // 
#define MOTOR_POW_L   10 // controlled by OC1B
#define MOTOR_POW_R   11 // controlled by OC2A
#define MOTOR_DIR_L   12 // 1 = forward, 0 = back
#define MOTOR_DIR_R   13 // 1 = forward, 0 = back
//LINE SENSOR HEX VALUES - hex value summarizing status of line followers //STICK INTO RESPFN
#define VALline0000 0x00
#define VALline000R 0x01
#define VALline00C0 0x02
#define VALline00CR 0x03
#define VALline0L00 0x04
#define VALline0L0R 0x05
#define VALline0LC0 0x06
#define VALline0LCR 0x07
#define VALlineF000 0x08
#define VALlineF00R 0x09
#define VALlineF0C0 0x10
#define VALlineF0CR 0x11
#define VALlineFL00 0x12
#define VALlineFL0R 0x13
#define VALlineFLC0 0x14
#define VALlineFLCR 0x15

/*---------------Helper Functions-----------------*/
/** Reads a string from Serial and
 *  casts it to an integer value
 */
int ReadSerialInt(void){
  String input = "";
  while (Serial.available()>0){
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      input += (char)inChar;
    }
  }
  return input.toInt();
}

/*---------------Helper Functions-----------------*/
void setup() {
  
  Serial.begin(9600);
}

/*---------------Helper Functions-----------------*/
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
  Serial.println("");
  
  switch (theKey) {
    case "A":
      // stuff
      break;
  }
}
