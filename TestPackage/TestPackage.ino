/*************************************************************
  File:      TestPrograms.ino
  Contents:  This program demonstrates simple control and 
  capabilities of the robot. Particularly, it is designed
  to act in response to a certain keystroke.
Commands:
1. "F100" Set flywheel speed                        GKH
 also a "W" to set flywheel power
2. "A" get flywheel speed                              GKH
3."B" print Beacon Pattern                            ZDR
4. "R" reverse drive train                               EDC
5. "G" go foward at set speed                       EDC
6. "I30" set thresh and turn on line following     EDC
7. "S" trigger solenoid to shoot                      GKH
8. "T30" turn some number of degrees         EDC
9. "E" Emergency stop of drivetrain              EDC
10. "P30" Pan servo                                     GKH
  Notes:     Target: Arduino Uno
             Arduino IDE version: 1.6.7
  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-01-09 LAK and ZDR  program created
  
 ************************************************************/
 
/*---------------Includes-----------------------------------*/
#include "robot_header.h"

/*---------------Variable Instantiation----------------------*/
// Timer Code
unsigned long current_time = 0;
// Beacon Sensor Object
BeaconSensor bSensor;
Shooter shooter;
// Drive timer
unsigned long drive_time = 0;
int alignment = 1;

/*---------------Setup-----------------*/
void setup() {
  
  Serial.begin(250000);
  // Beacon Code
  pinMode(BEACON1,INPUT); // set up beacon sensor pin
  pinMode(BEACON2,INPUT); // set up beacon sensor pin
  
  // Servo Code
  pinMode(SERVO_PAN, OUTPUT);
  ICR1   = 0x1800;
  TCCR1A = 0b10000010;
  TCCR1B = 0b00011010;
  OCR1A = 0x0BB8;
  
  //DriveMotor Code
  pinMode(MOTOR_POW_L, OUTPUT);
  pinMode(MOTOR_DIR_L, OUTPUT); 


  pinMode(MOTOR_POW_R, OUTPUT);
  pinMode(MOTOR_DIR_R, OUTPUT); 
  TCCR2A = 0b00000011;
  TCCR2B = 0b00000101; 

  //SHOOTING code
  pinMode(FLYWHEEL, OUTPUT);
  pinMode(SOLENOID, OUTPUT);

}

//#define CLIP_SENSE        A0 // clip sensor
//#define LINE_BACK_SENSE   A1 // back line sensor
//#define LINE_RIGHT_SENSE  A2 // right line sensor
//#define LINE_CENTER_SENSE A3 // center line sensor
//#define LINE_LEFT_SENSE   A4 // left line sensor
//#define FLYWHEEL_SENSE    A5 // Flywheel speed sensor
//
//// DIGITAL PINS
//#define SOLENOID       2 // 
//#define FLYWHEEL       3 // 
//#define BEACON1        5 //
//#define BEACON2        6 // 
//#define SERVO_PAN      9 // 
//#define MOTOR_POW_L   10 // controlled by OC1B
//#define MOTOR_POW_R   11 // controlled by OC2A
//#define MOTOR_DIR_L   12 // 1 = forward, 0 = back
//#define MOTOR_DIR_R   13 // 1 = forward, 0 = back

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
    case 'L': {
      // TAKE A SHOT!
      Serial.print("Taking a shot at speed=");
      unsigned int velocity = ReadSerialInt();
      Serial.println(velocity);
      shooter.shoot(velocity);
      }

      break;
    
    case 'F': {
      // SET FLYWHEEL SPEED
      // This function sets the flywheel speed. It is triggered by an input to the serial monitor of the form "F100", where the F tells the program to set the flywheel and the int tells us the speed on a map from 0 to 256 (for example).
      // Responsible: George Herring
      Serial.println("Modifying Motor Speed");
      unsigned int velocity = ReadSerialInt();
      shooter.setSpeed(velocity);
      Serial.println(velocity);
      }
      
      break;
      
      case 'A':{
        // Returns the current flywheel speed as an int between 0 and 256 (suggestion)
        // Responsible: George Herring
        Serial.print("Fly Wheel Speed is: ");
        Serial.println(getMotorSpeed());
        }
        break;
      
      case 'B':
        // Begins a sensor sweep, collects data for valid Beacon Pattern
        // prints out all beacon values (180)
        bSensor.findBeacons();
        break;
      
      case 'H':
        {// Computes the current heading, expects a valid sensor reading available
        int head = bSensor.getHeading(ReadSerialInt());
        Serial.print("Heading = ");
        Serial.println(head);
        }
        break;
      
      case 'R':
        // Reverse. Drives the drivetrain backward at a set speed. Stops after 5 seconds.
        motorLBack();
        motorRBack();
        break;
      
      case 'G':
        // Drives the drivetrain foward at a set speed. 
        motorLForward();
        motorRForward();
        break;
      
       case 'I':
        // GO forward and try to align on centerline.
        {
          motorLForward(); 
          motorRForward(); 
          drive_time = millis();
          alignment = 0;
        }
        break;

      case 'Z': {
        // go backward via line following
        // Responsible: Erica Chin
        int inLoadingZone = 0;
        unsigned int catchCounter = 0; 
        while (!inLoadingZone && catchCounter < 10){
          if (testForLine()) inLoadingZone = respLineFollow(1);
          catchCounter += 1; 
        }
        }
        break;
      
      case 'Y': {
        // go forward while line following
        for (unsigned int catchCounter = 0; catchCounter < 100; catchCounter++){
          if(testForLine()) respLineFollow(0); 
        }
        }
        break;

      
      case 'S':
        // trigger solenoid to shoot
        shooter.shootAdditional();
        break;
      
      case 'T':
        // turn drivetrain by some number of degrees
        { int angle1 = ReadSerialInt();
        botRotate(angle1);}
        break;
      
      case 'E':
        // emergency stop. Stop the motors from running. 
        // Responsible: Erica Chin
        stopDriveMotors();
        break;
      
      case 'P':{      // Number of Degrees for Angle
        bSensor.setAngle(ReadSerialInt());}
        break;
      
  }
}

/*---------------Main Loop-----------------*/
void loop() {
 
  // Grab the current time
  current_time = millis();
  
  // Handle serial input
  if (TestForKey()) RespToKey();
  
  // Update the Beacon Sensor
  bSensor.beaconUpkeep(current_time);

  // flywheel upkeep
  shooter.shooterUpkeep(current_time);

  // Line Following upkeep
  testForLine();

  unsigned int analogVal = 112;
  char inputChar;
  //Serial.println(otherVal);

  // handle line following
  if (!alignment) {
    alignment = respLineAlign();
  }
}
