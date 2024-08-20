// Auto-configures an Arduino 101 to act as an IMU to feed data to OpenTrack
//------------------------------------------------------------

#include <CurieIMU.h>
#include <MadgwickAHRS.h>


int js = 0;
float ax, ay, az;
float offset;
float gx, gy, gz;
Madgwick filter;

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

typedef struct
{
  int16_t Begin;  // 2  Debut
  uint16_t Cpt;   // 2  Compteur trame or Code info or error
  float gyro[3];  // 12 [Y, P, R]    gyro
  float acc[3];   // 12 [x, y, z]    Acc
  int16_t End;    // 2  Fin
} type_hat;

type_hat hat;
float roll, pitch, heading;

void FT_Data() {
  hat.Cpt++;
  if (hat.Cpt > 1000) {
    hat.Cpt = 0;
  }
  hat.Begin = 0xAAAA;
  hat.End = 0x5555;
  hat.gyro[0] = gx;
  hat.gyro[1] = gy;
  hat.gyro[2] = gz;

  hat.acc[0] = ax;
  hat.acc[1] = ay;
  hat.acc[2] = az;

  hat.gyro[0] = roll;
  hat.gyro[1] = pitch;
  hat.gyro[2] = heading;

  Serial.write((byte*)&hat, 30);
}

float convertRawAcceleration(int aRaw) {

  // since we are using 2G range

  // -2g maps to a raw value of -32768

  // +2g maps to a raw value of 32767



  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

float convertRawGyro(int gRaw) {

  // since we are using 250 degrees/seconds range

  // -250 maps to a raw value of -32768

  // +250 maps to a raw value of 32767



  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

void Init() {
  CurieIMU.autoCalibrateGyroOffset();

  Serial.println(" Done");

  Serial.print("Starting Acceleration calibration and enabling offset compensation...");

  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);

  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);

  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");
}
void setup() {
  Serial.begin(115200);

  // start the IMU and filter

  CurieIMU.begin();

  CurieIMU.setGyroRate(25);

  CurieIMU.setAccelerometerRate(25);

  // Set the accelerometer range to 2G

  CurieIMU.setAccelerometerRange(2);

  // Set the gyroscope range to 250 degrees/second

  CurieIMU.setGyroRange(250);
  Init();

  // initialize variables to pace updates to correct rate
  filter.begin(25);
  Sense();
  microsPerReading = 1000000 / 25;

  microsPrevious = micros();
}
void Sense() {
  int aix, aiy, aiz;

  int gix, giy, giz;
  // read raw data from CurieIMU

  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units

  ax = convertRawAcceleration(aix);

  ay = convertRawAcceleration(aiy);

  az = convertRawAcceleration(aiz);

  gx = convertRawGyro(gix);

  gy = convertRawGyro(giy);

  gz = convertRawGyro(giz);

  filter.updateIMU(gx, gy, gz, ax, ay, az);
}

void UpdatePose() {
  // print the heading, pitch and roll

  roll = filter.getRoll();

  pitch = filter.getPitch();

  heading = filter.getYaw();

  //IMU Outputs 0-360 coordinates, but endpoint wants -180-180 range.
  if (true) { heading = 180 - heading; }
}
void DebugPrint() {
  Serial.print("Orientation: ");

  Serial.print(heading);

  Serial.print(" ");

  Serial.print(pitch);

  Serial.print(" ");

  Serial.println(roll);
}
void loop() {
  unsigned long microsNow;

  // check if it's time to read data and update the filter

  microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading) {
    Sense();
    UpdatePose();
    FT_Data();

    // increment previous time, so we keep proper pace

    microsPrevious = microsPrevious + microsPerReading;
  }



  delay(10);
}
