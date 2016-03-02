// 
// PREPROCESSOR
// 
// Beacon Sensing
#define read_length 180
#define read_time 10  // Time for short move
#define wait_time 500 // Time for rapid move
#define read_pin 13
// ???
#define analogPin 3

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
  OCR1B = 0x0BB8-(angle<<4)+1440;
  OCR1A = 0x0BB8-(angle<<4)+1820;
}

// Serial Functions

/** Reads a string from Serial and
 *  casts it to an integer value
 *  Note: 1500 is neutral 2060 is 180 max
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


// 
// CLASSES
// 

class BeaconSensor {
private:
  bool dir_  = 1; // slew direction; 0->neg, 1->pos
  bool slew_ = 0; // 
  unsigned int idx_ = 0;   // angle index
  unsigned long last_ = 0; // last time
  bool read_master_[read_length];
  bool read_current_[read_length];
  bool scanning_ = 0;
  bool valid_read_ = 0;
  bool scan_start_ = 0;

  /** Sweeps servo angle between bounds.
   */
  void sweepAngle() {
    // Low bound
    if (idx_==0) {
      dir_ = 1;
      idx_ = 1;
    }
    // High bound
    else if (idx_==read_length-1) {
      dir_ = 0;
      idx_ = read_length-2;
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
    // TODO -- Initialize read_master_
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
        if (time - last_ > wait_time) {
          scan_start_ = 0;
          last_ = millis();
        }
      }
      // if we've started, do the sweep
      else {
        if (time - last_ > read_time) {
          // Take reading at current sensor angle
          read_current_[idx_] = digitalRead(read_pin);
          // Sweep the servo idx_
          sweepAngle();
          // Reset timer
          last_ = millis();
        }
      }
    }
    // Update servo set point to angle
    setServoAngle(idx_*180/read_length);
    // DEBUG -- print to serial
    Serial.print("[");
    Serial.print(idx_);
    Serial.print(",");
    Serial.print(scanning_);
    Serial.println("]");
  }

  /** Begins a new sensor sweep
   */
  void findBeacons() {
    scanning_ = 1;
    scan_start_ = 1;
    idx_ = 0;
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
BeaconSensor bSensor;

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
  pinMode(read_pin,INPUT); // set up beacon sensor pin
  
  // Servo Code
  pinMode(9, OUTPUT);
  ICR1   = 0x1000;//Set max counter value to 1024 in hex
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011010;
  OCR1A = 0x0200;
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
    if (res == 1) {
      bSensor.findBeacons();
    }
  }
}
