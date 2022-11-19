#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to GNSS
#include<Servo.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS


// PID Constants:
const double LINEAR_PARTIAL = 1/3.14159;  // used during honing phase (when trajectory towards target is ideally linear)
const double LINEAR_INTEGRAL = 0;
const double LINEAR_DERIVATIVE = 0;

const double APPROACH_PARTIAL = 1;  // used during final approach phase (when trajectory is ideally a circle around target)
const double APPROACH_INTEGRAL = .1;
const double APPROACH_DERIVATIVE = .25;

const bool INVERT_PID = false;  // use this to invert right/left servo tensioning


// MISC CONSTANTS
const int SERVO_BOUNDS[2] = {600, 2400};


// Configuration Constants:
String logFilePrefix = "log";
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
  tensionerServo.write(0);
  Serial.println("Actuated servo?");
  logFile = initLog(CHIP_SELECT, logFilePrefix, ".txt");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  initGPS();
  getCoordinates(previousCoordinates);
}//setup()



// init methods

// passed file path should not include the .txt, set that as fileLabel
// may be in a deep directory (will create if necessary)
File initLog(int chipSelect, String filePath, String fileLabel){
  filePath.toUpperCase();  // for some reason SD breakout defaults to all caps
  fileLabel.toUpperCase();
  Serial.print("Initializing SD card reader...");
  pinMode(chipSelect, OUTPUT);
  delay(100);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("initialization failed! (BLOCKING THREAD)");
    while (1);
  }//if
  Serial.println("initialization done.");
  int fileIndex = filePath.lastIndexOf("/")+1;
  Serial.print("Creating log file...");
  String dirPath = "/";
  if(fileIndex != -1){
    dirPath += filePath.substring(0,fileIndex-1);  // -1 to exclude the slash
    if(!SD.exists(dirPath)){
      SD.mkdir(dirPath);
    }//if
    Serial.print("created directory...");
  }//if
  File currentDir = SD.open(dirPath);
  int latestVer = 0;
  bool isValid;
  do{
    File currentFile = currentDir.openNextFile();
    isValid = currentFile == true;
    if(isValid){
      String fileName = currentFile.name();
      if(fileName.indexOf(filePath) == 0){  // in other words, the file we're looking at starts with filePath
        int numberEndIndex = fileName.indexOf(fileLabel);
        String versionStr = fileName.substring(filePath.length(), numberEndIndex);
        // now we have to make sure it's purely numeric so we know this is in fact the same file just with a version number
        bool isNum = true;
        for(int index = 0; index<versionStr.length() && isNum; index+=1){
          isNum = isDigit(versionStr.charAt(index));  // essentially breaks if this ever becomes false
        }//for
        if(isNum){
          int currentVer = versionStr.toInt();  // returns 0 if it's not a valid int
          // would only be invalid in cases like log1.txt vs. logFile0.txt (since it would try to convert File0 to int)
          // however, because it's 0, we don't have to worry about this interfering with valid files
          if(currentVer > latestVer){
            latestVer = currentVer;
          }//if
        }//if
      }//if
      currentFile.close();
    }//if
  } while(isValid);  // while it's a valid file
  String newFile = filePath+String(latestVer)+fileLabel;
  File fileObj = SD.open(newFile, FILE_WRITE);
  if(!fileObj){
    Serial.println("file object initialization failed! (BLOCKING THREAD)");
    while (1);
  }//if
  Serial.println("created file "+newFile);
  return fileObj;
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
// returns servo value (double) within [-1,1] based on
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
  if(pid>1){
    pid = 1.0;
  }//if
  else if(pid<-1){
    pid = -1.0;
  }//elif
  return pid;
}//linearPID()


// input should be within [-1,1]
void servoActuate(double position){
  int microseconds = (SERVO_BOUNDS[1]-SERVO_BOUNDS[0])*(position+1)/2 + SERVO_BOUNDS[0];
  tensionerServo.writeMicroseconds(microseconds);
}//servoActuate


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
    double alpha = trackingAngleError(targetCoordinates, coords, previousCoordinates);
    double rawPIDVal = linearPID(alpha, coords[4]);
    Serial.println("Error angle:"+String(alpha)+" rawPID:"+String(rawPIDVal));
    servoActuate(rawPIDVal);
    copyTo(coords, previousCoordinates, COORDINATES_LENGTH);
  }//if
  delay(1000);
}//loop
