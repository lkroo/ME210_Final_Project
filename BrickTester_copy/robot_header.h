// 
// COMPILER DEFINITIONS
// 

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

#define motorLSpeed 0x1800//0x138b
#define motorRSpeed 0xFF//0xd8

// BEACON SENSING
#define READ_LENGTH 180
#define READ_TIME 15  // Time for short move
#define CONV_THRESH 1 // Threshold for a valid signal sweep
#define IDX2DEG(idx) idx*180/READ_LENGTH // convert id to angle

// Servo Handling
#define SERVO_OFFSET_ANGLE 11
#define VALlightTopThreshold 300
#define VALClipLightThreshold 300

// SHOOTING
// required flywheel speed proportion to take a shot
#define SPEED_ERROR_THRESH 3

// 
// MISCELLANEOUS
// 
#define GAME_TIME 120000

unsigned long currentTime(unsigned long startTime){
  unsigned long timeElapsed = millis() - startTime;
  return timeElapsed;
}

// 
// DRIVE CODE
// 

void motorLForward(void){
  if (digitalRead(MOTOR_DIR_L) && ((OCR1B == motorLSpeed) && (TCCR1A & 0b00100000))){
    return;
  }
  else {
    digitalWrite(MOTOR_DIR_L  , HIGH);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = motorLSpeed;
    return;
  }
}
void motorRForward(void){
  if (digitalRead(MOTOR_DIR_R) && ((OCR2A == motorRSpeed) && (TCCR2A & 0b10000000))){
    return;
  }
  else {
    digitalWrite(MOTOR_DIR_R, HIGH);         
    TCCR2A = TCCR2A | 0b10000000;
    OCR2A = motorRSpeed;
  }
}

void motorLBack(void){
  if (!digitalRead(MOTOR_DIR_L) && ((OCR1B == motorLSpeed) && (TCCR1A & 0b00100000))){
    return;
  }
  else {
    digitalWrite(MOTOR_DIR_L  , LOW);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = motorLSpeed;
  }
}
void motorRBack(void){
  if (!digitalRead(MOTOR_DIR_R) && ((OCR2A == motorRSpeed) && (TCCR2A & 0b10000000))){
    return;
  }
  else {
    digitalWrite(MOTOR_DIR_R, LOW);         
    TCCR2A = TCCR2A | 0b10000000;
    OCR2A = motorRSpeed;
  }
}

void stopDriveMotors(void){
    TCCR1A = TCCR1A & 0b11011111; 
    TCCR2A = TCCR2A & 0b01111111; 
}

void botRotate(int deg){
  stopDriveMotors();
  unsigned long startTime = millis();
  unsigned long finalTime = abs(deg) * 9.63;
  while(currentTime(startTime)<finalTime){
    digitalWrite(MOTOR_POW_L, HIGH);
    digitalWrite(MOTOR_POW_R, HIGH);
      
    if (deg>0){
      digitalWrite(MOTOR_DIR_L, LOW);
      digitalWrite(MOTOR_DIR_R, HIGH);
    }
    else{
      digitalWrite(MOTOR_DIR_L, HIGH);
      digitalWrite(MOTOR_DIR_R, LOW);
    }
    //Serial.println(currentTime(startTime));
  }
  digitalWrite(MOTOR_POW_L, LOW);
  digitalWrite(MOTOR_POW_R, LOW);
}

// UNORGANIZED
unsigned char isClipEmpty(void){
  unsigned char trigger = (analogRead(PINclipSense)>VALClipLightThreshold);
  return trigger; 
}

// 
// LINE FOLLOWING
// 

static unsigned char VARsharedByte;

void setSharedInfoTo(unsigned char newByte){
  VARsharedByte = newByte;
}

unsigned char getSharedByte(void){
  return VARsharedByte;
}
unsigned char parked(void){
  unsigned char EventOccurred;
  unsigned char trigger = 0x00; //LCR
  unsigned int lightValC = 0;
  unsigned int lightValL = 0;
  unsigned int lightValR = 0; 
  
  lightValR = analogRead(PINlineSenseR);
  lightValC = analogRead(PINlineSenseC);
  lightValL = analogRead(PINlineSenseL);
  
  trigger = ((lightValL  >= VALlightTopThreshold)<<4)|((lightValC >=  VALlightTopThreshold)<<2)|(lightValR >= VALlightTopThreshold);
  
  EventOccurred = (trigger == VALline0LCR);
  return EventOccurred;  
}

unsigned char testForLine(void){
  unsigned char EventOccurred;
  unsigned char trigger = 0x00;  //BLCR
  static unsigned char lastTrigger = 0x00; 
    
  unsigned int lightValR = 0; // stores information from phototransistor
  unsigned int lightValC = 0;
  unsigned int lightValL = 0;
  unsigned int lightValF = 0;
  
  lightValF = analogRead(PINlineSenseF);
  lightValR = analogRead(PINlineSenseR);
  lightValC = analogRead(PINlineSenseC);
  lightValL = analogRead(PINlineSenseL);
  
  trigger = ((lightValF >= VALlightTopThreshold)<<6)|((lightValL  >= VALlightTopThreshold)<<4)|((lightValC >=  VALlightTopThreshold)<<2)|(lightValR >= VALlightTopThreshold);
  
  EventOccurred = ((trigger != 0x00) && (trigger != lastTrigger));
  if (trigger != lastTrigger) {
    setSharedInfoTo(trigger);
    Serial.print("line detected info:");
    Serial.print(lastTrigger);
  }
  lastTrigger = trigger;
  return EventOccurred;
}

unsigned char respLineAlign(void){
    unsigned char trigger;
    trigger = getSharedByte();
    //Serial.println(trigger, HEX);
    switch(trigger){
      //if the center has hit the line, then bot rotates clockwise
      case(VALline00C0): botRotate(-10); break; 
      case(VALline0LC0): botRotate(-10); break;
      case(VALline0LCR): botRotate(-10); break;
      case(VALline00CR): botRotate(-10); break; 

      case(VALline0000): motorRForward(); motorLForward(); break;
      case(VALlineF000): motorRForward(); motorLForward(); break;
           
      case(VALline000R): botRotate(-10); break;
      case(VALline0L00): botRotate(-10); break;
      case(VALline0L0R): botRotate(-10); break;

      case(VALlineFL0R): stopDriveMotors(); Serial.println("Center Sensor Error"); break;
      case(VALlineF00R): motorLForward(); break;
      case(VALlineFL00): motorRForward(); break;
      

      //if center and front are true, then we're aligned
      case(VALlineF0C0): stopDriveMotors(); Serial.println("DONE"); return 1; break;
      case(VALlineFLC0): stopDriveMotors(); Serial.println("DONE"); return 1; break;
      case(VALlineFLCR): stopDriveMotors(); Serial.println("DONE"); return 1; break;
      case(VALlineF0CR): stopDriveMotors(); Serial.println("DONE"); return 1; break;


              
      default: Serial.println("Error (respLineAlign()) broke."); motorRForward(); motorLForward(); break;
    }
    return 0;
}


// Drives robot to follow a line forward or backward
// returns 0 if robot is in loading zone
// returns 1 otherwise

unsigned char respLineFollow(unsigned int toBeacon){
    unsigned char trigger;
    trigger = getSharedByte();
    if (trigger == 0000){
      Serial.println("Bot is in white space?");
      return 0; //if we are in white space, return no line detected (0)
    }

    //Determine which direction we want to head, 
    //and store that information into the front 
    //line sensor bit in 'trigger'.
    if (toBeacon){
        //if toHome is 1, go to seesaws, 1***
        trigger = trigger & 1111;
        }
    else{
        //if toHome is 0, go to home beacon, 0***
        trigger = trigger & 0111;
        }
        
    switch(trigger){ //FORWARD CASES NEED TO BE FIXED DO NOT ENTER 1
        //forward
        case(VALlineF0C0): motorRForward(); motorLForward(); break;
        case(VALlineF000): stopDriveMotors(); break;                  
            case(VALlineFLCR): motorRForward(); motorLForward(); break;        
            case(VALlineFL0R): motorRForward(); motorLForward(); break; 
 
        //forward right
        case(VALlineF0CR): stopDriveMotors(); motorLForward(); break;
        case(VALlineF00R): stopDriveMotors(); motorLForward(); break;
        
        //forward left
        case(VALlineFLC0): stopDriveMotors(); motorRForward(); break;
        case(VALlineFL00): stopDriveMotors(); motorLForward(); break;
        
        //back
        case(VALline00C0): motorRBack(); motorLBack(); break;
        case(VALline00CR): motorRBack(); motorLBack(); break; 
        case(VALline0LC0): motorRBack(); motorLBack(); break;           
            case(VALline0L0R): motorRBack(); motorLBack(); Serial.println("Weird edge case 0L0R"); break; 
            
        //back right
        case(VALline000R): botRotate(-10); break;
        
        //back left
        case(VALline0L00):  botRotate(10); break;
        
        //stop
        case(VALline0LCR): stopDriveMotors(); break;
        case(VALline0000): stopDriveMotors(); break;
        
        
        
        default: Serial.println("I broked. line follow blah."); stopDriveMotors(); break;
    }
    return 1; 
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


// Serial Functions

/** Reads a string from Serial and
 *  casts it to an integer value
 */
int ReadSerialInt(void){
  String input = "";
  short signFlag = 1;
  while (Serial.available()>0){
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      input += (char)inChar;
    } 
    else if (inChar == '-'){
      signFlag = -1;
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
// SHOOTER
// 

unsigned int getMotorSpeed(void){
  return analogRead(FLYWHEEL_SENSE);
}

void setFlyWheelPower(unsigned int motorPower){
  if (motorPower==0){
    TCCR2A = TCCR2A & 0b11011111; 
  }
  else{
    TCCR2A = TCCR2A | 0b00100000;
    OCR2B = motorPower;
  }
}

int setFlyWheelSpeed(unsigned int Speed){
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
//  Serial.print(Speed);
//  Serial.print(",");
//  Serial.print(speedError);
  //Serial.print(",");
  //Serial.println(currentSpeed);
  setFlyWheelPower(speedError);
  return speedError;
}

class Shooter {
private:
  unsigned int velocity         = 0;
  uint8_t shooting              = 0;
  unsigned int speed_error      = 0;
  uint8_t shots_left            = 7;
  unsigned int stableSpeedCount = 0;

public:
  /** Upkeep function for shooter
   *  Drives flywheel to velocity setpoint,
   *  handles shot logic
   */
  void shooterUpkeep(unsigned long time) {
    // Set the flywheel speed
    speed_error = setFlyWheelSpeed(velocity);
    // Take a shot if able
    if (shooting){
      //Serial.println(speed_error, DEC);
      //Serial.println((velocity>>SPEED_ERROR_THRESH), DEC);
      if (speed_error <= (velocity>>SPEED_ERROR_THRESH)) {
        stableSpeedCount++;
        Serial.print("count");
        Serial.println(stableSpeedCount);
      }
      // TODO -- use timer instead of machine cycle
      if (stableSpeedCount>25){
        stableSpeedCount = 0;
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
  }
  /** Tells shooter to take a shot at a 
   *  prescribed flywheel speed
   * 
   * @param set_speed Desired flywheel speed
   *                  for shot
   */
  void shoot(unsigned int set_speed) {
    if (shooting == 0) {
      velocity = set_speed;
      shooting = 1;}
  }

  void shootAdditional(void){
    shooting = 1;
  }

  void reload(int val) {
    shots_left = val;
  }

  /** Returns the number of shots left
   *  in our magazine
   */
  uint8_t shotsLeft() {
    return shots_left;
  }

  /** 
   */
  int getShooting(){ 
    return shooting;
  }

  /** DEBUG -- Set flywheel speed setpoint
   */
  void setSpeed(unsigned int set_speed) {
    velocity = set_speed;
    //shooting = 0;
  }

  /** DEBUG -- Get flywheel speed setpoint
   */
  unsigned int getSpeed() {
    return velocity;
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
  bool edge_finding_ = 0; // e

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
    // Corner case
    for (int i=0; i<READ_LENGTH-1; ++i) {
              read_master_[i] = 0;
            }
    read_master_[33] = 1;
    read_master_[34] = 1;
    read_master_[35] = 1;
    read_master_[36] = 1;
    read_master_[37] = 1;
    read_master_[38] = 1;
    read_master_[39] = 1;

    read_master_[43] = 1;
    read_master_[44] = 1;
    read_master_[45] = 1;
    read_master_[46] = 1;
    read_master_[47] = 1;
    read_master_[48] = 1;
    read_master_[49] = 1;
    read_master_[50] = 1;
    read_master_[51] = 1;

    read_master_[59] = 1;
    read_master_[60] = 1;
    read_master_[61] = 1;
    read_master_[62] = 1;
    read_master_[63] = 1;
    read_master_[64] = 1;

    read_master_[78] = 1;
    read_master_[79] = 1;
    read_master_[80] = 1;
    read_master_[81] = 1;
    read_master_[82] = 1;
    read_master_[83] = 1;
    read_master_[84] = 1;
    read_master_[85] = 1;
    read_master_[86] = 1;
    read_master_[87] = 1;
    read_master_[88] = 1;

    // Center case
    for (int i=0; i<READ_LENGTH-1; ++i) {
              read_master2_[i] = 0;
            }
    read_master2_[41] = 1;
    read_master2_[42] = 1;
    read_master2_[43] = 1;
    read_master2_[44] = 1;
    read_master2_[45] = 1;
    read_master2_[46] = 1;
    read_master2_[47] = 1;
    read_master2_[48] = 1;
    read_master2_[49] = 1;
    read_master2_[50] = 1;
    read_master2_[51] = 1;
    read_master2_[52] = 1;
    read_master2_[53] = 1;
    read_master2_[54] = 1;
    read_master2_[55] = 1;
    read_master2_[56] = 1;
    read_master2_[57] = 1;
    read_master2_[58] = 1;
    read_master2_[59] = 1;
    read_master2_[60] = 1;
    read_master2_[61] = 1;
    read_master2_[62] = 1;
    read_master2_[63] = 1;
    read_master2_[64] = 1;
    read_master2_[65] = 1;
    read_master2_[66] = 1;
    read_master2_[67] = 1;

    read_master2_[74] = 1;

    read_master2_[76] = 1;
    read_master2_[77] = 1;
    read_master2_[78] = 1;
    read_master2_[79] = 1;
    read_master2_[80] = 1;
    read_master2_[81] = 1;
    read_master2_[82] = 1;
    read_master2_[83] = 1;
    read_master2_[84] = 1;
    read_master2_[85] = 1;
    read_master2_[86] = 1;
    read_master2_[87] = 1;
    read_master2_[88] = 1;
    read_master2_[89] = 1;
    read_master2_[90] = 1;
    read_master2_[91] = 1;
    read_master2_[92] = 1;
    read_master2_[93] = 1;
    read_master2_[94] = 1;
    read_master2_[95] = 1;
    read_master2_[96] = 1;
    read_master2_[97] = 1;
    read_master2_[98] = 1;
    read_master2_[99] = 1;
    read_master2_[100] = 1;
    read_master2_[101] = 1;
    read_master2_[102] = 1;
    read_master2_[103] = 1;
    read_master2_[104] = 1;
    read_master2_[105] = 1;
    read_master2_[106] = 1;
    read_master2_[107] = 1;
    read_master2_[108] = 1;

    read_master2_[118] = 1;
    read_master2_[119] = 1;
    read_master2_[120] = 1;
    read_master2_[121] = 1;
    read_master2_[122] = 1;
    read_master2_[123] = 1;
    read_master2_[124] = 1;
    read_master2_[125] = 1;
    read_master2_[126] = 1;
    read_master2_[127] = 1;
    read_master2_[128] = 1;
    read_master2_[129] = 1;
    read_master2_[130] = 1;
    read_master2_[131] = 1;

    read_master2_[141] = 1;
    read_master2_[142] = 1;
    read_master2_[143] = 1;
    read_master2_[144] = 1;
    read_master2_[145] = 1;
    read_master2_[146] = 1;
    read_master2_[147] = 1;
    read_master2_[148] = 1;
    read_master2_[149] = 1;
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
          read_current_[idx_] = digitalRead(BEACON1);
          // DEBUG -- print values
          // Serial.print("[");
          // Serial.print(idx_);
          // Serial.print(",");
          // Serial.print(read_current_[idx_]);
          // Serial.println("]");

          // Sweep the servo idx_
          sweepAngle();
          // Reset timer
          last_ = millis();
          
        }
      }
      // End scan
      if (idx_==0) {
        scanning_ = 0;
        valid_read_ = 1;
      }
    }

    // Update servo set point to angle
    setServoAngle(IDX2DEG(idx_));

  }

  /** Begins a new sensor sweep
   */
  void findBeacons() {
    if (scanning_ ==0 ) {
    scanning_   = 1;
    scan_start_ = 1;
    }
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

  int lEdge(){
    for (int i = 179; i>=0; i--){
      if(read_current_[i]){
        return i;//note: this return assumes that that there is one degree per index
      }
    }
  }
};

void setServoPos(unsigned int angle){
  //Input an angle between 0 and 180 as an unsigned int
  //Angle is off by 8 degrees
  unsigned int adjustedAngle = 0;
  adjustedAngle = angle + SERVO_OFFSET_ANGLE;
  OCR1B = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
  OCR1A = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
}


void arcLeft(void){
  if (!(digitalRead(MOTOR_DIR_L) && ((OCR1B == (motorLSpeed<<1)) && (TCCR1A & 0b00100000)))){
    digitalWrite(MOTOR_DIR_L  , HIGH);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = motorLSpeed>>1;
  }
  if (!(digitalRead(MOTOR_DIR_R) && ((OCR2A == motorRSpeed) && (TCCR2A & 0b10000000)))){
    digitalWrite(MOTOR_DIR_R, HIGH);         
    TCCR2A = TCCR2A | 0b10000000;
    OCR2A = motorRSpeed;
  }
  return;
}

void arcLeftBack(void){
  if (!(!digitalRead(MOTOR_DIR_L) && ((OCR1B == (motorLSpeed<<1)) && (TCCR1A & 0b00100000)))){
    digitalWrite(MOTOR_DIR_L  , LOW);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = motorLSpeed>>1;
  }
  if (!(!digitalRead(MOTOR_DIR_R) && ((OCR2A == motorRSpeed) && (TCCR2A & 0b10000000)))){
    digitalWrite(MOTOR_DIR_R, LOW);         
    TCCR2A = TCCR2A | 0b10000000;
    OCR2A = motorRSpeed;
  }
  return;
}

void arcRight(void){
  if (!(digitalRead(MOTOR_DIR_L) && ((OCR1B == motorLSpeed) && (TCCR1A & 0b00100000)))){
    digitalWrite(MOTOR_DIR_L  , HIGH);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = motorLSpeed;
  }
  if (!(digitalRead(MOTOR_DIR_R) && ((OCR2A == (motorRSpeed<<1)) && (TCCR2A & 0b10000000)))){
    digitalWrite(MOTOR_DIR_R, HIGH);         
    TCCR2A = TCCR2A | 0b10000000;
    OCR2A = motorRSpeed>>1;
  }
  return;
}

void arcRightBack(void){
  if (!(!digitalRead(MOTOR_DIR_L) && ((OCR1B == motorLSpeed) && (TCCR1A & 0b00100000)))){
    digitalWrite(MOTOR_DIR_L  , LOW);    
    TCCR1A = TCCR1A | 0b00100000;
    OCR1B = motorLSpeed;
  }
  if (!(!digitalRead(MOTOR_DIR_R) && ((OCR2A == (motorRSpeed<<1)) && (TCCR2A & 0b10000000)))){
    digitalWrite(MOTOR_DIR_R, LOW);         
    TCCR2A = TCCR2A | 0b10000000;
    OCR2A = motorRSpeed>>1;
  }
  return;
}

int seekCenterLine(bool beaconVal){
  unsigned char trigger;
  trigger = getSharedByte();
  //Serial.println(trigger, HEX);
  switch(trigger){
    //if the center has hit the line, then bot rotates clockwise
    case(VALline00C0): botRotate(-5); break; 
    case(VALline0LC0): botRotate(-5); break;
    case(VALline0LCR): botRotate(-5); break;
    case(VALline00CR): botRotate(-5); break; 

    case(VALline0000): 
      if(beaconVal){
        arcLeft();
      }
      else{
        arcRight();
      }
      break;

    case(VALlineF000): 
      if(beaconVal){
        arcLeft();
      }
      else{
        arcRight();
      }
      break;
         
    case(VALline000R): botRotate(-5); break;
    case(VALline0L00): botRotate(-5); break;
    case(VALline0L0R): botRotate(-5); break;

    case(VALlineFL0R): stopDriveMotors(); return 1; break;
    case(VALlineF00R): stopDriveMotors(); return 1; break;
    case(VALlineFL00): stopDriveMotors(); return 1; break;
    

    //if center and front are true, then we're aligned
    case(VALlineF0C0): stopDriveMotors(); return 1; break;
    case(VALlineFLC0): stopDriveMotors(); return 1; break;
    case(VALlineFLCR): stopDriveMotors(); return 1; break;
    case(VALlineF0CR): stopDriveMotors(); return 1; break;


            
    default: Serial.println("Error (respLineAlign()) broke."); motorRForward(); motorLForward(); break;
  }
  return 0;
}


int backupLine(void){
  unsigned char trigger;
  trigger = getSharedByte();
  //Serial.println(trigger, HEX);
  switch(trigger){
    //if the center has hit the line, then bot rotates clockwise
    case(VALline00C0): motorRBack(); motorLBack(); break; 
    case(VALline0LC0): arcLeftBack(); break;
    case(VALline00CR): arcRightBack(); break; 

    case(VALline0000): motorRBack(); motorLBack();break;

    case(VALlineF000): arcRightBack(); break;
         
    case(VALline000R): arcRightBack(); break;
    case(VALline0L00): arcLeftBack(); break;
    case(VALline0L0R): motorRBack(); motorLBack(); break;

    case(VALlineFL0R): stopDriveMotors(); break;
    case(VALlineF00R): arcRightBack(); break;
    case(VALlineFL00): arcLeftBack(); break;
    

    //if center and front are true, then we're aligned
    case(VALlineF0C0): motorRBack(); motorLBack(); break;
    case(VALlineFLC0): arcLeftBack(); break;
    case(VALlineFLCR): stopDriveMotors(); return 1; break;
    case(VALline0LCR): stopDriveMotors(); return 1; break;
    case(VALlineF0CR): arcRightBack(); break;


            
    default: Serial.println("Error (respLineAlign()) broke."); motorRForward(); motorLForward(); break;
  }
  return 0;
}

int forwardLine(void){
  unsigned char trigger;
  trigger = getSharedByte();
  //Serial.println(trigger, HEX);
  switch(trigger){
    //if the center has hit the line, then bot rotates clockwise
    case(VALline00C0): motorRForward(); motorLForward(); break; 
    case(VALline0LC0): motorRForward(); motorLForward(); break;
    case(VALline00CR): motorRForward(); motorLForward(); break; 

    case(VALline0000): motorRForward(); motorLForward();break;

    case(VALlineF000): motorRForward(); motorLForward(); break;
         
    case(VALline000R): arcRight(); break;
    case(VALline0L00): arcLeft(); break;
    case(VALline0L0R): motorRForward(); motorLForward(); break;

    case(VALlineFL0R): stopDriveMotors(); break;
    case(VALlineF00R): arcRight(); break;
    case(VALlineFL00): arcLeft(); break;
    

    //if center and front are true, then we're aligned
    case(VALlineF0C0): motorRForward(); motorLForward(); break;
    case(VALlineFLC0): arcLeft(); break;
    case(VALlineFLCR): motorRForward(); motorLForward(); break;
    case(VALline0LCR): motorRForward(); motorLForward(); break;
    case(VALlineF0CR): arcRight(); break;


            
    default: Serial.println("Error (respLineAlign()) broke."); motorRForward(); motorLForward(); break;
  }
  return 0;
}
