// 
// PREPROCESSOR
// 
// Beacon Sensing
#define READ_LENGTH 180
#define READ_TIME 20  // Time for short move
#define READ_PIN 5
#define CONV_THRESH 1 // Threshold for a valid signal sweep
#define idx2deg(idx) idx*180/READ_LENGTH // convert id to angle
// Servo Handling
#define servoOffsetAngle 11
//
// FUNCTIONS
//

// Servo Functions

/** Sets the servo position
 * @param angle unsigned int
 * @pre 0<=angle<=180
 */
void setServoAngle(unsigned int angle){
  //Input an angle between 0 and 180 as an unsigned int
  //Angle is off by 11 degrees
  unsigned int adjustedAngle = 0;
  adjustedAngle = angle + servoOffsetAngle;
  OCR1B = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
  OCR1A = 0x0BB8-(adjustedAngle<<3)-(adjustedAngle<<2)+1080;
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

class BeaconSensor {
private:
  bool dir_  = 1; // slew direction; 0->neg, 1->pos
  bool slew_ = 0; // 
  unsigned int idx_ = 0;   // angle index
  unsigned long last_ = 0; // last time
  // short read_master_[READ_LENGTH];  // sensor read at heading zero
  uint8_t read_master_[READ_LENGTH];
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
  BeaconSensor(int my_case) {
    // TODO -- Initialize read_master_
    switch (my_case) {
      case 1: // DEBUG MAP -- single beacon
        for (int i=0; i<READ_LENGTH-1; ++i) {
          if ((i>=57) or (i<=83)) {
            read_master_[i] = 1;
          }
          else {
            read_master_[i] = 0;
          }
        }
      break;
    }
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
          read_current_[idx_] = digitalRead(READ_PIN);
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
    setServoAngle(idx2deg(idx_));

    // DEBUG -- print to serial
    // Serial.print("[");
    // Serial.print(idx_);
    // Serial.print(",");
    // Serial.print(scanning_);
    // Serial.println("]");
  }

  /** Begins a new sensor sweep
   */
  void findBeacons() {
    scanning_   = 1;
    scan_start_ = 1;
  }

  /** Returns approximate heading
   *  based on sensor scan. Angle
   *  is given in terms of absolute
   *  frame, positive from the vertical
   *  axis, as defined in Erica's 
   *  coordinate system
   * 
   * return heading in [0,360]
   * return -1 if estimate unreliable
   */
  int getHeading() {
    // Convolve current read against master
    convolve(read_master_,read_current_,read_conv_);
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
      return fixAngle(idx2deg(READ_LENGTH-ind));
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

//
// GLOBAL VARIABLES
//
// DEBUG
unsigned int serialInt;

// Timer Code
unsigned long current_time = 0;


// Beacon Sensor
BeaconSensor bSensor(1); // debug case

// 
// SETUP
// 
void setup() {
  //
  // PINS
  //
  // DEBUG
  Serial.begin(250000);

  // Beacon Code
  pinMode(READ_PIN,INPUT); // set up beacon sensor pin
  
  // Servo Code
  pinMode(9, OUTPUT);
  ICR1   = 0x1800;
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011010;
  OCR1A = 0x0BB8;
}

// 
// MAIN LOOP
// 
void loop() {
  // Grab the current time
  current_time = millis();
  // Update the Beacon Sensor
  bSensor.beaconUpkeep(current_time);
  // DEBUG -- Respond to serial input
  if (Serial.available()>0) {
    int res = ReadSerialInt();
    // Do a scan
    if (res == 1) {
      bSensor.findBeacons();
    }
    // Compute the heading
    if (res == 2) {
      int head = bSensor.getHeading();
      Serial.print("Heading = ");
      Serial.println(head);
    }
  }
}

