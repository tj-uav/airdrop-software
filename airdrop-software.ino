#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to GNSS
#include<Servo.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

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
const int MIN_SATELLITES = 4;  // must have at least this many satellites for coordinates to be valid
const int IMU_LOG_ITERATIONS = 5;
const int GPS_LOG_ITERATIONS = 1;


// Configuration Constants:
String logFilePrefix = "bike/log";  // TODO: change to flight file for flight
const int CHIP_SELECT = 10;
const int SERVO_SIGNAL_PIN = 3;
const int STATUS_LED_PIN = 4;

const int GPS_QUERY_DELAY = 250;  // to prevent overloading I2C
const int IMU_QUERY_DELAY = 50;  // TODO: make this 50
const int COORDINATES_LENGTH = 5;
const int HEADING_LENGTH = 2;

long targetCoordinates[3] = {385288790, -777377410, 0};  // TODO: set coordinates


// Device Global Objects:
SFE_UBLOX_GNSS myGNSS;
Adafruit_BNO055 bno;
Servo tensionerServo;
File logFile;


// helper global vars
int last_gps_query_time = 0;
int gps_log_counter = 0;
int last_imu_query_time = 0;
int imu_log_counter = 0;
long integral;  // PID integral adder variable
long previousPartial;  // PID previous partial
long previousT;  //  time of previous measurement
long previousCoordinates[COORDINATES_LENGTH];
double initialHeading[HEADING_LENGTH];
double desiredAngleDelta = 0;


// setup

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  delay(1000);
  logStr("--------------- INITIALIZATION STARTED -------------------");
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // while (!Serial) {  // TODO: comment this part out before flight, don't want to wait for serial comms when not connected to computer
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }//while
  Serial.println("Started arduino");

  logFile = initLog(CHIP_SELECT, logFilePrefix, ".txt"); // TODO: uncomment
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  tensionerServo.attach(SERVO_SIGNAL_PIN);
  logStr("Attached servo.\n");

  initGPS();
  getCoordinates(&previousCoordinates[0]);

  initIMU();
  delay(1000);
  getHeadingVector(&initialHeading[0]);
  Serial.println("Heading x:" + String(initialHeading[0]) + ", y:" + String(initialHeading[1]));

  logStr("--------------- INITIALIZATION FINISHED ------------------");
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(5000);
  digitalWrite(STATUS_LED_PIN, LOW);
}//setup()



///////////////////////////////// init methods //////////////////////////////////////////

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
  String rawFilePrefix = filePath;
  bool newDir = false;
  if(fileIndex != 0){
    dirPath += filePath.substring(0,fileIndex);
    rawFilePrefix = filePath.substring(fileIndex, filePath.length());
    if(!SD.exists(dirPath)){
      SD.mkdir(dirPath);
      newDir = true;
      Serial.print("created directory...");
    }//if
  }//if
  Serial.print("directory: "+dirPath+"...");
  File currentDir = SD.open(dirPath);
  int latestVer = -1;
  bool isValid;
  if(!newDir){  // if it's not a new dir, we have to check for existing log files to start numbering
    do{
      File currentFile = currentDir.openNextFile();
      isValid = currentFile == true;
      if(isValid){
        String fileName = currentFile.name();
        if(fileName.indexOf(rawFilePrefix) == 0){  // in other words, the file we're looking at starts with filePath
          int numberEndIndex = fileName.indexOf(fileLabel);
          String versionStr = fileName.substring(rawFilePrefix.length(), numberEndIndex);
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
  }//if
  Serial.print("found version: "+String(latestVer)+"...");
  latestVer += 1;  // we want to make it the next version
  String newFile = filePath+String(latestVer)+fileLabel;
  File fileObj = SD.open(newFile, FILE_WRITE);
  if(!fileObj){
    Serial.println("file object initialization failed! (BLOCKING THREAD)");
    while (1);
  }//if
  Serial.println("created file "+newFile+".");
  return fileObj;
}//initSD()


void initGPS(){
  logStr("Initializing GPS...");
  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    logStr(F("\nu-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }//if

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  last_gps_query_time = millis(); 
  logStr("finished GPS init.\n");
}//initGPS()


void initIMU(){
  bno = Adafruit_BNO055(55, 0x28, &Wire);
  logStr("Initializing IMU...");
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    logStr("failed! (BLOCKING THREAD)\n");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  logStr("finished IMU init.\n");
}//initIMU



///////////////////////////////// device interfacing ////////////////////////////////////////////////
void logStr(String data){
  logFile.print(data);  
  Serial.print(data);
  logFile.flush(); 
}//logStr()


// input should be within [-1,1]
void servoActuate(double position){
  int microseconds = (SERVO_BOUNDS[1]-SERVO_BOUNDS[0])*(position+1)/2 + SERVO_BOUNDS[0];
  tensionerServo.writeMicroseconds(microseconds);
}//servoActuate


// Populates empty input array with: [lat, long, alt, satellites, measurement_time]
// NOTE: satellites is also used as a status indicator
//   0 means no satellites but i2c comms completed
//   -1 means GPS_QUERY_DELAY milliseconds have not passed since the last query, so didn't even check
//   and likely inaccurate unless satellites >= 3
void getCoordinates(long* toPopulate){
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  toPopulate[4] = last_gps_query_time = millis(); //Update the time
  toPopulate[0] = myGNSS.getLatitude();
  toPopulate[1] = myGNSS.getLongitude();
  toPopulate[2] = myGNSS.getAltitude();
  toPopulate[3] = myGNSS.getSIV();

  if(toPopulate[3] >= MIN_SATELLITES){
    digitalWrite(STATUS_LED_PIN, HIGH);
  }//if
  else{
    digitalWrite(STATUS_LED_PIN, LOW);
  }//else
  
  if(gps_log_counter % GPS_LOG_ITERATIONS == 0){  // to only log with enough: toPopulate[3] >= MIN_SATELLITES && 
    gps_log_counter = 0;
    String gps_str = "";
    logStr(gps_str);
    if (myGNSS.getDateValid()){
      gps_str += "Date:"+String(myGNSS.getYear())+"-"+String(myGNSS.getMonth())+"-"+String(myGNSS.getDay());
    }//if
    if (myGNSS.getTimeValid()){
      gps_str += ", Time:"+String(myGNSS.getHour())+":"+String(myGNSS.getMinute())+":"+String(myGNSS.getSecond());
    }//if
    gps_str += ", Lat:"+String(toPopulate[0])+", Long:"+String(toPopulate[1])+", Alt:"+String(toPopulate[2])+", Satellites:"+String(toPopulate[3])+"\n";
    logStr(gps_str);
  }//if
  gps_log_counter +=1;
}//getCoordinates()


// LENGTH 3
void getGravVector(double* toPopulate){
    imu::Vector<3> gravVector = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    toPopulate[0] = gravVector.x();
    toPopulate[1] = gravVector.y();
    toPopulate[2] = gravVector.z();
}//getGravVector


// LENGTH 3
void getMagVector(double* toPopulate){
    imu::Vector<3> magVector = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    toPopulate[0] = magVector.x();
    toPopulate[1] = magVector.y();
    toPopulate[2] = magVector.z();
}//getGravVector


// Returns a normalized supposed heading vector (it will actually be 90 degrees off of true heading, but this still allows us to compute delta angle while preserving efficiency)
// LENGTH 2
void getHeadingVector(double* toPopulate){
    double gravity[3];
    double magnetic[3];
    getGravVector(&gravity[0]);
    getMagVector(&magnetic[0]);
    if(imu_log_counter % IMU_LOG_ITERATIONS == 0){
      logStr("Gravity -- x:"+String(gravity[0])+", y:"+String(gravity[1])+", z:"+String(gravity[2])+"\n");
      logStr("Magnetic -- x:"+String(magnetic[0])+", y:"+String(magnetic[1])+", z:"+String(magnetic[2])+"\n");
    }//if
    buildHeadingVector(toPopulate, gravity, magnetic);
    imu_log_counter += 1;
    if(imu_log_counter % IMU_LOG_ITERATIONS == 0){
      imu_log_counter = 0;
    }//if
}//getHeadingVector


// gets the new GPS coordinates, updates global vars, and returns the desired heading change
double desiredHeadingDelta(long* targetCoords){
  long currentCoordinates[COORDINATES_LENGTH];
  getCoordinates(&currentCoordinates[0]);
  double value = trackingAngleError(&targetCoords[0], &currentCoordinates[0], &previousCoordinates[0]);
  copyTo(&currentCoordinates[0], &previousCoordinates[0], COORDINATES_LENGTH);
  double heading[HEADING_LENGTH];
  getHeadingVector(&heading[0]);
  copyTo(&heading[0], &initialHeading[0], HEADING_LENGTH);
  return value;
}//desiredHeadingDelta


// gets the new GPS coordinates, updates global vars, and returns the desired heading change
double gpsHeadingAngle(){
  long northHeading[2] = {0, 1};
  long currentCoordinates[COORDINATES_LENGTH];
  getCoordinates(&currentCoordinates[0]);
  long vx = currentCoordinates[1] - previousCoordinates[1];  // velocity x (east,west) component
  // vx is actually this times r/dt, but whatever, magnitudes cancel later anyway
  long vy = currentCoordinates[0] - previousCoordinates[0];  // velocity y (north,south) component
  double alpha = vectorAngle(northHeading[0], northHeading[1], vx, vy);
  copyTo(&currentCoordinates[0], &previousCoordinates[0], COORDINATES_LENGTH);
  return alpha;
}//gpsHeadingAngle

/////////////////////////////// Functional helpers //////////////////////////////////////////
// Should all be purely functional!! no device interfacing or global vars

// returns radian angle between two vectors
//  sign: positive if the a vector is to the right of the b vector (i.e. a x b > 0), else negative
// therefore, return value is within (-pi,pi]
double vectorAngle(double ax, double ay, double bx, double by){
  // Serial.println("ax:"+String(ax)+" ay:"+String(ay)+" bx:"+String(bx)+" by:"+String(by));
  double alpha = abs(acos(
    (bx*ax + by*ay) / (sqrt(sq(bx) + sq(by)) * sqrt(sq(ax) + sq(ay)))
  )); // absolute value( arccos( dot product of {desired, current} trajectory unit vectors ) )
  
  if( ax*by - ay*bx < 0){
    // if target_vector cross-product velocity < 0, the desired trajectory is to the left of the current trajectory, so negate alpha:
    alpha *= -1;  
  }//if 
  return alpha;
}//vectorAngle



// populates array of length 2 with a 2d heading vector based on two, length 3, arrays of doubles: gravitaitonal field and magnetic field
void buildHeadingVector(double* toPopulate, double* gravitational, double* magnetic){
    double gx = gravitational[0];
    double gy = gravitational[1];
    double gz = gravitational[2];
    double nx = magnetic[0];
    double ny = magnetic[1];
    double nz = magnetic[2];

    double px = gy*nz - gz*ny;
    double py = gz*nx - gx*nz;
    double pz = gx*ny - gy*nx;

    double npx = py*gz - pz*gy;
    double npy = pz*gx - px*gz;
    double npz = px*gy - py*gx;

    double p = sqrt(px*px + py*py + pz*pz);
    double np = sqrt(npx*npx + npy*npy + npz*npz);

    if(imu_log_counter % IMU_LOG_ITERATIONS == 0){
      logStr("z vector -- G:"+String(gz/ sqrt(sq(gx)+ sq(gy) + sq(gz)))+", Np:"+String(npz/np)+", P:"+String(pz/p)+"\n");
    }//if
    toPopulate[0] = -pz/p;
    toPopulate[1] = npz/np;
}//buildHeadingVector


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
  double alpha = vectorAngle(tx, ty, vx, vy);
  return alpha;
}//trackingAngleError()


void copyTo(long* values, long* target, int length){
  for(int i = 0; i < length; i++){
    target[i] = values[i];
  }//for
}//copyTo

void copyTo(double* values, double* target, int length){
  for(int i = 0; i < length; i++){
    target[i] = values[i];
  }//for
}//copyTo


/////////////////////////////////////// PID ////////////////////////////////////////////////////////////////

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


void linearIteration(){

}//linearIteration()


// loop

void loop() {
  double time1 = millis();
  if(time1 - last_gps_query_time > GPS_QUERY_DELAY){
    last_gps_query_time = time1;
    logStr("heading-angle:"+String(gpsHeadingAngle())+"\n");
  }//if
  double time2 = millis();  // refresh our time just in case that took long
  if(time2 - last_imu_query_time > IMU_QUERY_DELAY){
    last_imu_query_time = time2;
    double heading[HEADING_LENGTH];
    getHeadingVector(&heading[0]);
    double alpha = vectorAngle(heading[0], heading[1], 0, 1);
    if(imu_log_counter % IMU_LOG_ITERATIONS == 0){
      logStr("angle:"+String(alpha)+"\n");
    }//if
  }//if
}//loop
