/*------ COMPILER DEFINITIONS --------*/
// ANALOG SENSORS
#define PINclipSense   A0 // clip sensor
#define PINlineSenseF  A1 // back line sensor
#define PINlineSenseR  A2 // right line sensor
#define PINlineSenseC  A3 // center line sensor
#define PINlineSenseL  A4 // left line sensor
#define FLYWHEEL_SENSE A5 // Flywheel speed sensor

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

//LINE SENSOR HEX VALUES - hex value summarizing status of line followers 
#define VALline0000 0x00
#define VALline000R 0x01
#define VALline00C0 0x04
#define VALline00CR 0x05
#define VALline0L00 0x10
#define VALline0L0R 0x11
#define VALline0LC0 0x14
#define VALline0LCR 0x15
#define VALlineF000 0x40
#define VALlineF00R 0x41
#define VALlineF0C0 0x44
#define VALlineF0CR 0x45
#define VALlineFL00 0x50
#define VALlineFL0R 0x51
#define VALlineFLC0 0x54
#define VALlineFLCR 0x55

#define motorLSpeed 0x138b
#define motorRSpeed 0xd8

// BEACON SENSING
#define READ_LENGTH 180
#define READ_TIME 20  // Time for short move
#define CONV_THRESH 3 // Threshold for a valid signal sweep
#define IDX2DEG(idx) idx*180/READ_LENGTH // convert id to angle

// Servo Handling
#define SERVO_OFFSET_ANGLE 11
#define VALlightTopThreshold 300
// SHOOTING
// required flywheel speed proportion to take a shot
#define SPEED_ERROR_THRESH 0.1

/*-------- LOGIC DEFINITIONS --------*/
// Dead-Reckoning angles (degrees)
#define DEAD_ANGLE_1 0
#define DEAD_ANGLE_2 90
// Dead-Reckoning times (milliseconds)
#define DEAD_TIME_1 1000
// FIRING ANGLES at SHOOTING ZONE (degrees)
#define SHOOT_ANGLE_1 -35
#define SHOOT_ANGLE_2 -17
#define SHOOT_ANGLE_3 0
#define SHOOT_ANGLE_4 17
#define SHOOT_ANGLE_5 35

// 
// FUNCTIONS
// 

// Servo Functions

void motorLForward(void){
    digitalWrite(MOTOR_DIR_L  , HIGH);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = 0x099d; 
}
void motorRForward(void){
  digitalWrite(MOTOR_DIR_R, HIGH);         
  TCCR2A = TCCR2A | 0b10000000;
  OCR2A = 0xeb;
}

void motorLBack(void){
    digitalWrite(MOTOR_DIR_L  , LOW);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = 0x099d; 
}
void motorRBack(void){
  digitalWrite(MOTOR_DIR_R, LOW);         
  TCCR2A = TCCR2A | 0b10000000;
  OCR2A = 0xeb;
}

void stopDriveMotors(void){
    TCCR1A = 0b00000010; 
    TCCR2A = 0b00000010; 
}

/** Sets the servo position
 * @param angle unsigned int
 * @pre 0<=angle<=180
 */
void setServoAngle(unsigned int angle){
  //Input an angle between 0 and 180 as an unsigned int
  //Angle is off by 11 degrees
  unsigned int adjustedAngle = 0;
  adjustedAngle = angle + SERVO_OFFSET_ANGLE;
  OCR1B = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
  OCR1A = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
}

void setServoPos(unsigned int angle){
  //Input an angle between 0 and 180 as an unsigned int
  //Angle is off by 8 degrees
  unsigned int adjustedAngle = 0;
  adjustedAngle = angle + servoOffsetAngle;
  OCR1B = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
  OCR1A = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
}

// Flywheel functions

void setFlyWheelPower(unsigned int motorPower){
  if (motorPower==0){
    TCCR2A = TCCR2A & 0b11011111;
  }
  else{
    TCCR2A = TCCR2A | 0b00100000;
    OCR2B = motorPower;
  }
}

void setFlyWheelSpeed(unsigned int Speed){
  //Speed in arbitary units where 200 is medium fast and 400 is very very fast
  //Don't go over 400
  unsigned int currentSpeed;
  int speedError;
  currentSpeed = getMotorSpeed();
  speedError = 3*(Speed - currentSpeed);
  if (speedError<0){
    speedError = 0;
  }
  else if (speedError>255){
    speedError = 255;
  }
  //Serial.print(Speed);
  //Serial.print(",");
  //Serial.print(speedError);
  //Serial.print(",");
  //Serial.println(currentSpeed);
  setFlyWheelPower(speedError);
}

unsigned int getMotorSpeed(void){
  return analogRead(flywheelSpeedPin);
}

// Serial Functions

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

// Helper functions

/** Convolution operator
 * 
 * @param Signal[READ_LENGTH]
 * @param Kernel[READ_LENGTH]
 * @param Result[READ_LENGTH*2-1]
 *
 * @post Result contains the convolution of 
 *       Signal and Kernel
 * 
 * Code via: http://stackoverflow.com/questions/8424170/1d-linear-convolution-in-ansi-c-code
 */
void convolve(const uint8_t Signal[],
              const uint8_t Kernel[],
              char Result[])
{
  unsigned long n;

  for (n = 0; n < READ_LENGTH + READ_LENGTH - 1; n++)
  {
    unsigned long kmin, kmax, k;

    Result[n] = 0;

    kmin = (n >= READ_LENGTH - 1) ? n - (READ_LENGTH - 1) : 0;
    kmax = (n < READ_LENGTH - 1) ? n : READ_LENGTH - 1;

    for (k = kmin; k <= kmax; k++)
    {
      Result[n] += Signal[k] * Kernel[n - k];
    }
  }
}

/** Set degree angle to be in [0,360]
 */
int fixAngle(int angle) {
  if (angle < 0) {
    return fixAngle(angle+360);
  }
  if (angle > 360) {
    return fixAngle(angle-360);
  }
}

// 
// CLASSES
// 

class Shooter {
private:
  unsigned int speed = 0;
  uint8_t shooting   = 0;
  int speed_error    = 0;
  uint8_t shots_left = 7;

public:
  /** Upkeep function for shooter
   *  Drives flywheel to velocity setpoint,
   *  handles shot logic
   */
  void shooterUpkeep(unsigned long time) {
    // Set the flywheel speed
    speed_error = setFlyWheelSpeed(speed);
    // Take a shot if able
    if (shooting && 
        (abs(speed_error) < speed*SPEED_ERROR_THRESH)) {
      // TODO -- FIX BLOCKING CODE!
      digitalWrite(SOLENOID, HIGH);
      delay(200);
      digitalWrite(SOLENOID, LOW);
      // Clear shot flag
      shooting = 0;
      // TODO -- Use the clip sensor to determine
      //         if the shot was successful
      // DEBUG -- Assume the shot worked
      --shots_left;
    }
  }

  /** Tells shooter to take a shot at a 
   *  prescribed flywheel speed
   * 
   * @param set_speed Desired flywheel speed
   *                  for shot
   */
  void shoot(unsigned int set_speed) {
    speed = set_speed;
    shooting = 1;
  }

  /** Returns the number of shots left
   *  in our magazine
   */
  uint8_t shotsLeft() {
    return shots_left;
  }

  /** DEBUG -- Set flywheel speed setpoint
   */
  void setSpeed(unsigned int set_speed) {
    speed = set_speed;
  }

  /** DEBUG -- Get flywheel speed setpoint
   */
  unsigned int getSpeed() {
    return speed;
  }
};

class BeaconSensor {
private:
  bool dir_  = 1; // slew direction; 0->neg, 1->pos
  bool slew_ = 0; // 
  unsigned int idx_ = 90 * 180/READ_LENGTH;   // angle index
  unsigned long last_ = 0; // last time
  // short read_master_[READ_LENGTH];  // sensor read at heading zero
  uint8_t read_master_[READ_LENGTH];  // Right corner
  uint8_t read_master2_[READ_LENGTH]; // Center
  uint8_t read_current_[READ_LENGTH]; // current sensor read
  char read_conv_[READ_LENGTH*2-1]; // convolved readings
  bool scanning_ = 0;     // currently scanning?
  bool valid_read_ = 0;   // 
  bool scan_start_ = 0;   // 

  /** Sweeps servo angle between bounds.
   */
  void sweepAngle() {
    // Low bound
    if (idx_==0) {
      dir_ = 1;
      idx_ = 1;
    }
    // High bound
    else if (idx_==READ_LENGTH-1) {
      dir_ = 0;
      idx_ = READ_LENGTH-2;
    }
    // Interior
    else {
      idx_ += dir_ - (1-dir_);
    }
  }

public:
  /** Public constructor
   */
  BeaconSensor() {
    // TODO -- Collect sensor readings at critical field positions
    for (int i=0; i<READ_LENGTH-1; ++i) {
              read_master_[i] = 0;
            }
    read_master_[32] = 1;
    read_master_[33] = 1;
    read_master_[34] = 1;
    read_master_[35] = 1;
    read_master_[36] = 1;

    read_master_[40] = 1;
    read_master_[41] = 1;
    read_master_[42] = 1;
    read_master_[43] = 1;
    read_master_[44] = 1;
    read_master_[45] = 1;
    read_master_[46] = 1;

    read_master_[52] = 1;
    read_master_[53] = 1;
    read_master_[54] = 1;
    read_master_[55] = 1;
    read_master_[56] = 1;
    read_master_[57] = 1;
    read_master_[58] = 1;
    read_master_[59] = 1;

    read_master_[70] = 1;
    read_master_[71] = 1;
    read_master_[72] = 1;
    read_master_[73] = 1;
    read_master_[74] = 1;

    read_master_[91] = 1;
    read_master_[92] = 1;
    read_master_[93] = 1;
    read_master_[94] = 1;
    read_master_[95] = 1;
    read_master_[96] = 1;
    read_master_[97] = 1;
    read_master_[98] = 1;
    read_master_[99] = 1;
    read_master_[100] = 1;
    read_master_[101] = 1;
  }

  /** Drives sweeps servo between angle bounds
   *  and takes beacon sensor measurements.
   */
  void beaconUpkeep(unsigned long time) {
    // No millis() overflow handling...

    // Scan 
    if (scanning_) {
      // if we're just starting, march to zero
      if (scan_start_) {
        if (idx_ < READ_LENGTH-1) {
          if (time - last_ > READ_TIME) {
            ++idx_;
            last_ = millis();
          }
        }
        else {
          scan_start_ = 0;
        }
        
      }
      // if we've started, do the sweep
      else {
        if (time - last_ > READ_TIME) {
          // Take reading at current sensor angle
          read_current_[idx_] = digitalRead(BEACON1 );
          // DEBUG -- print values
          Serial.print("[");
          Serial.print(idx_);
          Serial.print(",");
          Serial.print(read_current_[idx_]);
          Serial.println("]");

          // Sweep the servo idx_
          sweepAngle();
          // Reset timer
          last_ = millis();
          
        }
      }
      // End scan
      if (idx_==0) {
        scanning_ = 0;
      }
    }

    // Update servo set point to angle
    setServoAngle(IDX2DEG(idx_));

  }

  /** Begins a new sensor sweep
   */
  void findBeacons() {
    scanning_   = 1;
    scan_start_ = 1;
  }
  
  /** Set new servo setpoint;
   *  overrides other rotation behavior
   */
  void setAngle(unsigned int angle) {
    idx_ = angle * 180 / READ_LENGTH;
    scanning_ = 0;
    scan_start_ = 0;
  }

  /** Returns approximate heading
   *  based on sensor scan. Angle
   *  is given in terms of absolute
   *  frame, positive from the vertical
   *  axis, as defined in Erica's 
   *  coordinate system
   * 
   * @param my_case switches the convolution kernel
   *        my_case == 1 center field
   *        my_case != 1 right corner
   * 
   * return heading in [0,360]
   * return -1 if estimate unreliable
   */
  int getHeading(int my_case) {
    // Convolve current read against master
    if (my_case==1) {
      convolve(read_master2_,read_current_,read_conv_);
    }
    else {
      convolve(read_master_,read_current_,read_conv_);
    }
    // Compute maximal element
    unsigned long sum = read_conv_[0];
    unsigned long ind = 0;
    for (unsigned long i=1; i<(READ_LENGTH*2-1); ++i) {
      if (read_conv_[i]>read_conv_[ind]) {
        ind = i;
      }
      sum += read_conv_[i];
    }
    // Check sum for valid signal strength
    if (sum<CONV_THRESH) {
      return -1;
    }
    // Compute heading
    else {
      return fixAngle(IDX2DEG(READ_LENGTH-ind));
    }
  }

  /** Returns the current scan
   */
  uint8_t* getInfo() {
    return read_current_;
  }

  /** Invalidates the current scan
   */
  void clear() {
    valid_read_ = 0;
  }

  /** Returns whether scanning
   */
  bool isScanning() const {
    return scanning_;
  }

  /** Returns whether current read
   *  is valid
   */
  bool isValid() const {
    return valid_read_;
  }
};

/*---------Variable Initialization-----------*/
// FORGIVE US OUR TRESPASSES, AS WE FORGIVE
// THOSE WHO TRESPASS AGAINST US.
uint8_t my_case = 1;
unsigned long current_time = 0;

// Beacon Sensor Object
BeaconSensor bSensor; // create new object

// Shooter Object
Shooter shooter(); // create new object

// Robot absolute heading
int bot_angle;

// Driving 
unsigned long drive_time1 = 0;

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
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011010;
  OCR1A = 0x0BB8;
  
  // Drive Motor Code
  pinMode(MOTOR_POW_L, OUTPUT);
  pinMode(MOTOR_DIR_L, OUTPUT); 
  pinMode(MOTOR_POW_R, OUTPUT);
  pinMode(MOTOR_DIR_R, OUTPUT); 
  TCCR2A = 0b10000011;
  TCCR2B = 0b00000101;

  // Shooter Code
  pinMode(SOLENOID, OUTPUT);
  pinMode(FLYWHEEL, OUTPUT);
}

// 
// MAIN LOOP
// 
void loop() {
    // Update main timer
    current_time = millis();
    
    // Check for game end
    if (current_time > GAME_TIME) {
        my_case = 13; // disable robot
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
    }
    // Run main loop logic
    switch my_case:
        case 1: // Initiating Beacon Sensor Sweep
            // Take a sensor sweep if we
            // have no valid sensor data
            if (!bSensor.isValid()) {
              bSensor.findbeacons();
            }
            else {
              // check if sensor reading 
              // doesn't meet threshold value
              if (bSensor.getHeading == -1) {
                // if beacons are not in
                // view, we should rotate 180 deg
                bSensor.clear();
                botRotate(120);
              }
              // otherwise, proceed
              else {
                ++my_case;
              }
            }
            break;
            
        case 2: // Find Bot Angle, take shots
                // on three closest beacons until the clip is empty
                if (shooter.shotsLeft() > 0) && ( isClipEmpty() )
            bot_angle = bSensor.getHeading(0); 
            ++my_case;
            break;
            
        case 3: // Turn to DEAD_ANGLE_1
            // TODO -- is botRotate() blocking?
            botRotate(DEAD_ANGLE_1-bot_angle);
            bSensor.clear(); // data is now invalid
            ++my_case;
            break;

        case 4: // Drive forward for DEAD_TIME_1
            if (drive_time1==0) {
              drive_time1 = millis();
              // TODO // botForward();
            }
            else if (current_time - drive_time1 > DEAD_TIME_1) {
              // Reset driver timer
              drive_time1 = 0;
              // TODO // botStop();
              ++my_case;
            }
            break;

        case 5: // Turn to DEAD_ANGLE_2
            // TODO -- is botRotate() blocking?
            botRotate(DEAD_ANGLE_2);
            ++my_case;
            break;

        case 6: // Drive until line intercept
            if (/*---Sensor Not Triggered---*/) {
              // TODO // botForward();
            }
            else {
              ++my_case;
            }
            break;

        case 7: // Turn CW until front sensor intercept
            // TODO - does respLineAlign() return 0 
            // while not aligned?
            if (!respLineAlign())
              ++my_case;
            break;

        case 8: // Line follow until T intersection
            // TODO - does respLineFollow() return 0
            // while not in loading zone?
            if (!respLineFollow()) {
              ++my_case;
            }
            break;

        case 9: // targeting
            // - define an expected orientation (0 deg)  and EXPECTED beacon pattern
            // - get current orientation by calling bsensor.getHeading()
            // - define orientationOffset = expected - current orientation.
            // - define an array of 5 integers which define angles of target beacons. 
            //   (beaconLocations = Expected + orientationOffset)
            // - case ++

            // Take a sensor sweep if we
            // have no valid sensor data
            if (!bSensor.isValid()) {
              bSensor.findbeacons();
            }
            // We have valid sensor data
            else {
              // Compute heading at shooting location
              bot_angle = bSensor.getHeading(1);
              ++my_case;
            }
            break;

        case 10: // firing
            // while clip is NOT empty:
            // - loop from i = 1:5
            //  1. move servo into angle(i)
            //  2. fire 1 chip
            // - case ++

        case 11: // reloading
            // - backup for set period of time (same as movement forward in case backingUpToFiringPosition).
            // - do nothing for set period of time
            // - move forward until 3 line sensors trigger
            // - initialize timer
            // - move straight forward until timer expires. Then stop. (this is to ensure the ENTIRE robot is outside of the reloading zone)
            // - change to case 8 (case 9?)
            
        case 13: // GameOver
            // - stop all systems.
            // - If 2 min expired, case = 13
}
