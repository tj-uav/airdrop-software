#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
double initialHeading[2] = {1,0};

void setup(void)
{
  Serial.begin(9600);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  getHeadingVector(&initialHeading[0]);
  delay(5000);
  Serial.println("Initial Heading -- x:"+String(initialHeading[0])+" y:"+String(initialHeading[1]));
  delay(1000);
}


// returns radian angle between two vectors
//  sign: positive if the a vector is to the right of the b vector (i.e. a x b > 0), else negative
// therefore, return value is within (-pi,pi]
double vectorAngle(double ax, double ay, double bx, double by){
  double alpha = abs(acos(
    (bx*ax + by*ay) / (sqrt(sq(bx) + sq(by)) * sqrt(sq(ax) + sq(ay)))
  )); // absolute value( arccos( dot product of {desired, current} trajectory unit vectors ) )
  
  if( ax*by - ay*bx < 0){
    // if target_vector cross-product velocity < 0, the desired trajectory is to the left of the current trajectory, so negate alpha:
    alpha *= -1;  
  }//if 
  return alpha;
}//vectorAngle


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

    // Serial.println("z vector -- G:"+String(gz/ sqrt(sq(gx)+ sq(gy) + sq(gz)))+", Np:"+String(npz/np)+", P:"+String(pz/p));

    toPopulate[0] = -pz/p;
    toPopulate[1] = npz/np;
}//buildHeadingVector


// Returns a normalized supposed heading vector (it will actually be 90 degrees off of true heading, but this still allows us to compute delta angle while preserving efficiency)
// LENGTH 2
void getHeadingVector(double* toPopulate){
  double gravity[3];
  double magnetic[3];
  getGravVector(&gravity[0]);
  getMagVector(&magnetic[0]);
  // Serial.println("Gravity -- x:"+String(gravity[0])+", y:"+String(gravity[1])+", z:"+String(gravity[2]));
  // Serial.println("Magnetic -- x:"+String(magnetic[0])+", y:"+String(magnetic[1])+", z:"+String(magnetic[2]));
  buildHeadingVector(toPopulate, gravity, magnetic);
}//getHeadingVector


void loop(void)
{
  delay(BNO055_SAMPLERATE_DELAY_MS);
  double heading[2];
  getHeadingVector(&heading[0]);
  // Serial.println("x:"+String(heading[0])+", y:"+String(heading[1]));
  Serial.println("radians:"+String(vectorAngle(0, 1, heading[0], heading[1])));
}//loop


