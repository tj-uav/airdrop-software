#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to GNSS
#include<Servo.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS


// PID Constants:
const long LINEAR_PARTIAL = 1;  // used during honing phase (when trajectory towards target is ideally linear)
const long LINEAR_INTEGRAL = .1;
const long LINEAR_DERIVATIVE = .25;

const long APPROACH_PARTIAL = 1;  // used during final approach phase (when trajectory is ideally a circle around target)
const long APPROACH_INTEGRAL = .1;
const long APPROACH_DERIVATIVE = .25;

const bool INVERT_PID = false;  // use this to invert right/left servo tensioning


// MISC CONSTANTS
const long GPS_LAT_CORRECTION =   
// this is added to the GPS's output to get the actual
// used to correct for the massive ~10 meter bias in the GPS's position


// Configuration Constants:
String logFileName = "log1.txt";
const int CHIP_SELECT = 10;
const int SERVO_SIGNAL_PIN = 3;

const int GPS_QUERY_DELAY = 250;  // to prevent overloading I2C
const int COORDINATES_LENGTH = 5;

long targetCoordinates[3] = {388173876, -771681275, 0};  // TODO MAKE THIS EASY TO SET


// Device Global Objects:
SFE_UBLOX_GNSS myGNSS;
Servo tensionerServo;
File logFile;


// helper global vars
int last_gps_query_time;
long integral;  // PID integral adder variable
long previousPartial;  // PID previous partial
long previousT;  //  time of previous measurement
long previousCoordinates[COORDINATES_LENGTH];


// setup

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }//while
  tensionerServo.attach(SERVO_SIGNAL_PIN);
  initSD(CHIP_SELECT);
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logFile = SD.open(logFileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (!logFile){
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }//if
  initGPS();
  previousCoordinates[0] = previousCoordinates[1] = previousCoordinates[2] = previousCoordinates[3] = previousCoordinates[4] = 0;
}//setup()



// init methods

void initSD(int chip_select){
  Serial.print("Initializing SD card reader...");
  pinMode(chip_select, OUTPUT);
  delay(100);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("initialization failed!");
    while (1);
  }//if
  Serial.println("initialization done.");
}//initSD()

void initGPS(){
  Serial.print("Initializing GPS...");
  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("\nu-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }//if

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  last_gps_query_time = millis(); 
  Serial.println("finished GPS init.");
}//initGPS()



// device interfacing helpers

void logStr(String data){
  logFile.print(data);
  logFile.flush();
}//logStr()


// Populates empty input array with: [lat, long, alt, satellites, measurement_time]
// NOTE: satellites is also used as a status indicator
//   0 means no satellites but i2c comms completed
//   -1 means GPS_QUERY_DELAY milliseconds have not passed since the last query, so didn't even check
//   and likely inaccurate unless satellites >= 3
void getCoordinates(long* toPopulate){
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - last_gps_query_time > GPS_QUERY_DELAY)
  {
    toPopulate[4] = last_gps_query_time = millis(); //Update the time
    toPopulate[0] = myGNSS.getLatitude();
    toPopulate[1] = myGNSS.getLongitude();
    toPopulate[2] = myGNSS.getAltitude();
    toPopulate[3] = myGNSS.getSIV();
  }//if
  else{
    toPopulate[3] = -1;
  }//else
}//getCoordinates()



// Calculation helpers

// given 3 arrays of [lat, long]  angles MUST BE given in same degree power (i.e. degrees*10^7, is what u-blox returns)
// returns error angle
//  magnitude: radian angle between the current velocity and the straight line path to the target coordinates
//  sign: positive if the target trajectory path is to the right of the current velocity (i.e. target_vector x velocity > 0), else negative
// therefore, return value is within (-pi,pi]
double trackingAngleError(long* target, long* current, long* previous){
  long vx = current[1] - previous[1];  // velocity x (east,west) component
  // vx is actually this times r/dt, but whatever, magnitudes cancel later anyway
  long vy = current[0] - previous[0];  // velocity y (north,south) component
  long tx = target[1] - current[1];  // straight line to target (desired trajectory) x component
  long ty = target[0] - current[0];  // desired trajectory y component
  String vectorStr = "vx: "+String(vx)+" vy: "+String(vy)+" tx: "+String(tx)+" ty: "+String(ty);
  Serial.print("Magnitude product"+String((sq(tx) + sq(ty)) * (sq(vx) + sq(vy)) ));
  double alpha = abs(acos(
    (tx*vx + ty*vy) / (sqrt(sq(tx) + sq(ty)) * sqrt(sq(vx) + sq(vy)))
  )); // absolute value( arccos( dot product of {desired, current} trajectory unit vectors ) )
  
  if( tx*vy - ty*vx < 0){
    // if target_vector cross-product velocity < 0, the desired trajectory is to the left of the current trajectory, so negate alpha:
    alpha *= -1;  
  }//if 
  Serial.println(vectorStr+" alpha: "+String(alpha));
  return alpha;
}//trackingAngleError()


// USED DURING THE INITIAL LINEAR TRAJECTORY PHASE
// returns servo value based on
//    alpha (radians): current trajectory's error angle (as returned by trackingAngleError)
//    currentT (milliseconds): time of current measurement, used for delta since last measurement (units are irrelevant because of constants, but just use milliseconds for consistency)
// updates previousT, previous, and integral global helper variables
double linearPID(double alpha, double currentT){
  // remember, partial = target - current, and target is 0 (which is where the angle between v and trajectory is 0)
  // therefore: partial = -alpha
  double currentPartial = -alpha;
  double deltaT = currentT - previousT;

  if(currentPartial * previousPartial < 0){  // if the signs of the current and previous angle differ
    // we must reset the integral summation
    integral = 0;  // we'll just set it to zero since we don't actually know when it last crossed
  }//if
  else{
    integral += currentPartial*deltaT;
  }//else 
  double pid = 
    currentPartial * LINEAR_PARTIAL  
    +  (currentPartial-previousPartial)/deltaT * LINEAR_DERIVATIVE  
    +  integral * LINEAR_INTEGRAL;  //double pid
  if(INVERT_PID){
    pid *= -1;
  }//if 
  previousPartial = currentPartial;
  previousT = currentT;
  return pid;
}//linearPID()


void linearIteration(long* current_coords){

}//linearIteration()


void copyTo(long* values, long* target, int length){
  for(int i = 0; i < length; i++){
    target[i] = values[i];
  }//for
}//copyTo


// loop

void loop() {
  long coords[COORDINATES_LENGTH];
  getCoordinates(&coords[0]);
  if(coords[3] >=3){
    String gps_str = "Lat: "+String(coords[0])+"(10^7deg) Long: "+String(coords[1])+"(10^7deg) Alt: "+String(coords[2])+"(mm) Satellites: "+String(coords[3])+"\n";
    logStr(gps_str);
    Serial.print("Logging:"+gps_str);
    trackingAngleError(targetCoordinates, coords, previousCoordinates);
    copyTo(coords, previousCoordinates, COORDINATES_LENGTH);
  }//if
  delay(1000);
}//loop
