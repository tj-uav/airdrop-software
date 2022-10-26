#include <SPI.h>
#include <SD.h>

String logFileName = "log.txt";
const int CHIP_SELECT = 10;

File logFile;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  initSd(CHIP_SELECT);
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logFile = SD.open(logFileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (logFile) {
    Serial.print("Writing to test.txt...");
    logStr("Test---------");
    logFile.close();
    Serial.println("done.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

}


void initSd(chip_select){
  Serial.print("Initializing SD card...");
  pinMode(chip_select, OUTPUT);
  delay(100);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
}


void logStr(String data){
  logFile.print(data);
  logFile.flush();
}


void loop() {
  // put your main code here, to run repeatedly:
  

}