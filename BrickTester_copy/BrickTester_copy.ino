#include "arduino.h"
#include "robot_header.h"

 /*---------Variable Initialization-----------*/
 // FORGIVE US OUR TRESPASSES, AS WE FORGIVE
 // THOSE WHO TRESPASS AGAINST US.
 //uint8_t my_case = 1;
 unsigned long current_time = 0;
 int desiredBotAngle = 30;
 // Beacon Sensor Object
 BeaconSensor bSensor; // create new object
 // Shooter Object
 Shooter shooter; // create new object
 // Robot absolute heading
 int bot_angle;
 // Driving 
 unsigned long drive_time1 = 0;
int alignment = 1;
int reload = 0;

// 
// SETUP
// 
void setup() {

  // DEBUG -- SERIAL
  Serial.begin(250000);

  // Beacon Code
  pinMode(BEACON1,INPUT); // set up beacon sensor pin
  pinMode(BEACON2,INPUT);
  
  // Pivot Servo Code
  pinMode(SERVO_PAN, OUTPUT);
  ICR1   = 0x1800;
  TCCR1A = 0b10000010;
  TCCR1B = 0b00011010;
  OCR1A = 0x0BB8;
  
  // Drive Motor Code
  pinMode(MOTOR_POW_L, OUTPUT);
  pinMode(MOTOR_DIR_L, OUTPUT); 
  pinMode(MOTOR_POW_R, OUTPUT);
  pinMode(MOTOR_DIR_R, OUTPUT); 
  TCCR2A = 0b00000011;
  TCCR2B = 0b00000101;

  // Shooter Code
  pinMode(SOLENOID, OUTPUT);
  pinMode(FLYWHEEL, OUTPUT);
}

// 
// MAIN LOOP
// 
void loop() {
    //
    // Serial.print("my_case=");
    // Serial.println(my_case);
    // Serial.print("heading");
    // Serial.println(bSensor.getHeading(0));
    // Update main timer
    static unsigned long startTimeInCase = 0;
    static int previousCase = 0;
    static int my_case = 1; // default
//     static int my_case = 5; // test intermediate
    if(previousCase!=my_case){
      Serial.print("In case:");
      Serial.println(my_case, DEC);
      startTimeInCase = millis();
      previousCase = my_case;
    }

    current_time = millis();
    
    
    // Check for game end
    if (current_time > GAME_TIME) {
        my_case = 13; // disable robot
        shooter.shoot(0); // stop flywheel
        stopDriveMotors();
        analogWrite(FLYWHEEL,0);
    }
    
    //////////////  Run upkeep  /////////////////
    else {
        // Beacon Sensor upkeep
        bSensor.beaconUpkeep(current_time);
        
        // flywheel upkeep
        shooter.shooterUpkeep(current_time);

        // Line Following upkeep
         testForLine();

        // handle line following
          if (!alignment) {
              alignment = respLineAlign();
           }
//          if (reload){
//            reload = respLineFollow();
//          }
      }
    // Run main loop logic
    switch (my_case){
        case 1: // Initiating Beacon Sensor Sweep
          // Take a sensor sweep if we
          // have no valid sensor data
          if (!bSensor.isValid()) {
            bSensor.findBeacons();
          }
          else {
            // check if sensor reading 
            // doesn't meet threshold value
            int temp_head = bSensor.getHeading(0);
            Serial.print("Heading=");
            Serial.println(temp_head);
            if ((temp_head == -1) || 
              ((temp_head>270)&&(temp_head<300))) {
              // if beacons are not in
              // view, we should rotate 180 deg
              bSensor.clear();
              botRotate(180);
            }
            // otherwise, proceed
            else {
              bot_angle = bSensor.getHeading(0);

              // run beacon test again to prepare for targeting routine
              bSensor.findBeacons();
              // TODO -- change to case 2
              my_case = 3;
            }
          }
          break;
            
        case 2: // Find Bot Angle, take shots
                // on three closest beacons until the clip is empty
          // if ((shooter.shotsLeft() > 0) && ( !isClipEmpty() )){
        if (shooter.shotsLeft() > 0){
            // if clip not empty then go into shooting mode
            bot_angle = bSensor.getHeading(0); //CHECK: is the 0 input of get heading correct??!
            
            int shotAngle1 = fixAngle(93 -(bot_angle));
            int shotAngle2 = fixAngle(95 -(bot_angle));
            int shotAngle3  = fixAngle(97-(bot_angle));
            unsigned int speed1 = 175;
            unsigned int speed2 = 175;
            unsigned int speed3 = 175;

            if ((shooter.shotsLeft() > 0) && 
                (!shooter.getShooting())) {
              //shoot at closest beacon                   
              bSensor.setAngle(shotAngle1); //pan servo
              shooter.shoot(speed1); //setflywheel speed & shoots
            }

            if ((shooter.shotsLeft()>3) && (shooter.shotsLeft() < 6)){
               // shoot at 2nd closest beacon
              bSensor.setAngle(shotAngle2); //pan servo
              shooter.shoot(speed2); //setflywheel speed & shoots
                   
            }

            if ((shooter.shotsLeft()>0) && (shooter.shotsLeft() < 4)){
               //shoot at 3rd closest Beacon
              bSensor.setAngle(shotAngle3); //pan servo
              shooter.shoot(speed3); //setflywheel speed & shoots              
              // DEBUG
              
            }                  
          }
          else {
            my_case = 4; // go to beacon line forward
            shooter.setSpeed(0);
            bSensor.setAngle(90);
          }
          break;
            
         case 3: // Align bot with far left beacon
              static int leftEdge = 0;
              if (bSensor.isValid()){
                leftEdge = 65-bSensor.getHeading(0);
                if (leftEdge>180){
                  leftEdge -= 360;
                }
                else if (leftEdge<-180){
                  leftEdge += 360;
                }
                if (abs(leftEdge)>9){
                  Serial.print(leftEdge, DEC);
                  botRotate(leftEdge);
                  bSensor.clear();
                }
                else{
                  my_case = 2; // go to shooting
                  // my_case = 4; // skip shooting
                  bSensor.setAngle(90);
                }
              }
              else{
                bSensor.findBeacons();
              }
              break;
        case 4: // Drive towards beacon until tape line hit
            static int foundLine = 0;
            foundLine = seekCenterLine(digitalRead(BEACON1));
            bSensor.clear();
            if(foundLine){
              my_case = 5;
            }
            break;
          
        case 5: // Rotate and back up on line
            static int inLoading = 0;
            inLoading = backupLine();
            // bSensor.clear();
            if(inLoading){
              my_case = 6;
            }
            break;

        case 6: // Loading (Waiting)
            //Serial.print(current_time-startTimeInCase, DEC);
            if (current_time-startTimeInCase>5000){
              my_case = 7;
              shooter.reload(7);
            }
            break;

        case 7: // Drive forward
            forwardLine();
            if (current_time-startTimeInCase>700){
              stopDriveMotors();
              my_case = 8;
            }
            break;

        case 8: // Sensor Sweep
            // wait until we have a valid read
            if (!bSensor.isValid()) {
              bSensor.findBeacons();
            }
            else {
               my_case = 9; // GO TO SHOOTING
//              my_case = 5; // DEBUG
            }
            break;

        case 9: // Shooting
          // Find Bot Angle, take shots
                // on three closest beacons until the clip is empty
          // if ((shooter.shotsLeft() > 0) && ( !isClipEmpty() )){
        if (shooter.shotsLeft() > 0){
            // if clip not empty then go into shooting mode
            bot_angle = bSensor.getHeading(1);
            
            int shotAngle1 = fixAngle(57 -(bot_angle));
            int shotAngle2 = fixAngle(90 -(bot_angle));
            int shotAngle3  = fixAngle(123-(bot_angle));
            unsigned int speed1 = 151;
            unsigned int speed2 = 150;
            unsigned int speed3 = 151;

            if ((shooter.shotsLeft() > 5) && 
                (!shooter.getShooting())) {
              //shoot at closest beacon                   
              bSensor.setAngle(shotAngle1); //pan servo
              shooter.shoot(speed1); //setflywheel speed & shoots
            }

            if ((shooter.shotsLeft()>3) && (shooter.shotsLeft() < 6)){
               // shoot at 2nd closest beacon
              bSensor.setAngle(shotAngle2); //pan servo
              shooter.shoot(speed2); //setflywheel speed & shoots
                   
            }

            if ((shooter.shotsLeft()>0) && (shooter.shotsLeft() < 4)){
               //shoot at 3rd closest Beacon
              bSensor.setAngle(shotAngle3); //pan servo
              shooter.shoot(speed3); //setflywheel speed & shoots              
              // DEBUG
              
            }                  
          }
          else {
            my_case = 5; // return to motor reverse case
            shooter.setSpeed(0);
          }
          break;

        // case 4: 
        //     if (alignment){                      
        //       motorLBack();
        //       motorRBack();
  
        //       if(parked()){
        //         stopDriveMotors();
        //         ++my_case;
        //       }
        //      }
        // case 5: //timing , move to case 6 once timing for reload is up
        //   if (!isClipEmpty()) {
        //     delay(1000);
        //     ++my_case;
        //     }
        //     break;

        // case 6:
        //   motorLForward(); 
        //   motorRForward(); 
        //   delay(100);
        //   stopDriveMotors();
        //   ++my_case;
        //   break;

        // case 7:
        //     static int shotsRemaining = 7;
        //     while(shotsRemaining>0){
        //       if (shooter.shotsLeft() > 0){
        //         // if clip not empty then go into shooting mode
        //         //bot_angle = bSensor.getHeading(0); //CHECK: is the 0 input of get heading correct??!
                
        //         int shotAngle1 = 90;
        //         int shotAngle2 = 57;
        //         int shotAngle3 = 123;
        //         unsigned int speed1 = 155;
        //         unsigned int speed2 = 160;
        //         unsigned int speed3 = 160;

        //         if ((shooter.shotsLeft() > 5) && 
        //             (!shooter.getShooting())) {
        //           //shoot at closest beacon
        //           bSensor.setAngle(shotAngle1); //pan servo
        //           shooter.shoot(speed1); //setflywheel speed & shoots
        //         }

        //         if ((shooter.shotsLeft()>3) && (shooter.shotsLeft() < 6)){
        //            // shoot at 2nd closest beacon
        //           bSensor.setAngle(shotAngle2); //pan servo
        //           shooter.shoot(speed2); //setflywheel speed & shoots
                       
        //         }

        //         if ((shooter.shotsLeft()>0) && (shooter.shotsLeft() < 4)){
        //            //shoot at 3rd closest Beacon
        //           bSensor.setAngle(shotAngle3); //pan servo
        //           shooter.shoot(speed3); //setflywheel speed & shoots              
        //           // DEBUG
                  
        //         }                  
        //       }
        //       else {
        //         my_case = 4;
        //         shooter.setSpeed(0);
        //       }
        //     }
        //     //else !! if we're out of chips            
        //     break;

        // case 5: // Turn to DEAD_ANGLE_2
        //     // TODO -- is botRotate() blocking?
        //     botRotate(DEAD_ANGLE_2);
        //     ++my_case;
        //     break;

        // case 6: // Drive until line intercept
        //     if (/*---Sensor Not Triggered---*/) {
        //       // TODO // botForward();
        //     }
        //     else {
        //       ++my_case;
        //     }
        //     break;

        // case 7: // Turn CW until front sensor intercept
        //     // TODO - does respLineAlign() return 0 
        //     // while not aligned?
        //     if (!respLineAlign())
        //       ++my_case;
        //     break;

        // case 8: // Line follow until T intersection
        //     // TODO - does respLineFollow() return 0
        //     // while not in loading zone?
        //     if (!respLineFollow()) {
        //       ++my_case;
        //     }
        //     break;

        // case 9: // targeting
        //     // - define an expected orientation (0 deg)  and EXPECTED beacon pattern
        //     // - get current orientation by calling bsensor.getHeading()
        //     // - define orientationOffset = expected - current orientation.
        //     // - define an array of 5 integers which define angles of target beacons. 
        //     //   (beaconLocations = Expected + orientationOffset)
        //     // - case ++

        //     // Take a sensor sweep if we
        //     // have no valid sensor data
        //     if (!bSensor.isValid()) {
        //       bSensor.findbeacons();
        //     }
        //     // We have valid sensor data
        //     else {
        //       // Compute heading at shooting location
        //       bot_angle = bSensor.getHeading(1);
        //       ++my_case;
        //     }
        //     break;

        // case 10: // firing
        //     // while clip is NOT empty:
        //     // - loop from i = 1:5
        //     //  1. move servo into angle(i)
        //     //  2. fire 1 chip
        //     // - case ++

        // case 11: // reloading
        //     // - backup for set period of time (same as movement forward in case backingUpToFiringPosition).
        //     // - do nothing for set period of time
        //     // - move forward until 3 line sensors trigger
        //     // - initialize timer
        //     // - move straight forward until timer expires. Then stop. (this is to ensure the ENTIRE robot is outside of the reloading zone)
        //     // - change to case 8 (case 9?)
            
        // case 13: // GameOver
        //     // - stop all systems.
        //     // - If 2 min expired, case = 13
        case 13:
          shooter.shoot(0); // stop flywheel
          stopDriveMotors();
          analogWrite(FLYWHEEL,0);
          break;
    }
}
