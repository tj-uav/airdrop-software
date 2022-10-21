#include <SPI.h>
#include <SD.h>


File logFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (logFile) {
    Serial.print("Writing to test.txt...");
    logFile.println("This is a test file :)");
    logFile.println("testing 1, 2, 3.");
    for (int i = 0; i < 20; i++) {
      logFile.println(i);
    }
    // close the file:
    logFile.close();
    Serial.println("done.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  

}