#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to GNSS
#include<Servo.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS


String logFileName = "log1.txt";
const int CHIP_SELECT = 10;
const int SERVO_SIGNAL_PIN = 3;


SFE_UBLOX_GNSS myGNSS;
Servo tensionerServo;
File logFile;


int LAST_GPS_QUERY_TIME;
int GPS_QUERY_DELAY = 250;  // to prevent overloading I2C


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  tensionerServo.attach(SERVO_SIGNAL_PIN);
  initSD(CHIP_SELECT);
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logFile = SD.open(logFileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (!logFile){
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  initGPS();

}


void initSD(int chip_select){
  Serial.print("Initializing SD card reader...");
  pinMode(chip_select, OUTPUT);
  delay(100);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}

void initGPS(){
  Serial.print("Initializing GPS...");
  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("\nu-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  LAST_GPS_QUERY_TIME = millis(); 
  Serial.println("finished GPS init.");
}


void logStr(String data){
  logFile.print(data);
  logFile.flush();
}


// Populates empty input array with: [lat, long, alt, satellites]
// NOTE: satellites is also used as a status indicator
// 0 means no satellites but i2c comms completed
// -1 means GPS_QUERY_DELAY milliseconds have not passed since the last query, so didn't even check
// and likely inaccurate unless satellites >= 3
void getCoordinates(long* toPopulate){
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - LAST_GPS_QUERY_TIME > GPS_QUERY_DELAY)
  {
    LAST_GPS_QUERY_TIME = millis(); //Update the timer
    toPopulate[0] = myGNSS.getLatitude();
    toPopulate[1] = myGNSS.getLongitude();
    toPopulate[2] = myGNSS.getAltitude();
    toPopulate[3] = myGNSS.getSIV();
  }
  else{
    toPopulate[3] = -1;
  }
}

void loop() {
  long coords[4];
  getCoordinates(&coords[0]);
  if(coords[3] >=3){
    String gps_str = "Lat: "+String(coords[0])+"(microdeg) Long: "+String(coords[1])+"(microdeg) Alt: "+String(coords[2])+"(mm) Satellites: "+String(coords[3])+"\n";
    logStr(gps_str);
    Serial.print("Logging:"+gps_str);
  }
}
