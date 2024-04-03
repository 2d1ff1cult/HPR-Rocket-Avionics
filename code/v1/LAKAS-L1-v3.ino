/*
LAKAS-1
a Tripoli Level 1 (L1) High-Power Rocket (HPR)

Code description:
Uses the following sensors:
- Altitude:
  - BMP388 altimeter (spec. barometric pressure sensor)
- Orientation, velocity, and angular velocities:
  - MPU6050 6DOF MEMS accelerometer + gyroscope
- Telemetry:
  - nRF24L01 + PA + LNA (power amplifier + low noise amplifier)
    - Operates at 250 KBPS
- Positioning
  - uBlox NEO-6M
    - really cool GPS device
- Logging
  - HW-125 SD breakout board
    - 5V to 3.3V LDO for easy usage
    - don't forget to format SD card to FAT32!

Rationale:
Get basic flight data from rocket during flight and send in real-time to ground station (aka my laptop)

Wiring:
BMP388    Nano
VIN       5V
GND       GND
SCK       A5
SDI       A4

MPU6050   Nano
VCC       5V
GND       GND
SCK       A5
SCL       A4

nRF24L01  Nano
VCC       5V (note: using adapter)
GND       GND
CE        9
CSN       10
SCK       13
MOSI      11
MISO      12

NEO-6M    Nano
VCC       external power
RX        4
TX        3
GND       GND

HW-125    Nano
VCC       5V
GND       GND
CS        5
MOSI      11
MISO      12
CLK       13

v3 Changelog:
03/19/2024 - launch was scrapped, moved to 4/20/2024
03/17/2024 - finally receiving GPS coords in this program; was missing checking if SoftwareSerial was available
03/19/2024 - verifed receipt on RX of GPS signals

v2 Changelog:
03/16/2024 - not working with GPS even though GPS examples working
03/16/2024 - updated receiver code, confirmed receiving
03/14/2024 - sd card causes nano to not work, still fixing

*/

#include <Wire.h> // I2C
#include <SPI.h> //SPI
#include <SoftwareSerial.h> // serial for GPS
#include <TinyGPS.h>  // TinyGPS for post processing

// sd
#include <SD.h>

// altimeter
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <bmp3.h>
#include <bmp3_defs.h>
#define SEALEVELPRESSURE_HPA (1013.25)

// accelgyro
#include "I2Cdev.h"
#include <MPU6050.h> // accel + gyro

// nRF24L01 libs
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#define CE_PIN   9
#define CSN_PIN 10

// sensor definitions
Adafruit_BMP3XX bmp; // defining a sensor called bmp
MPU6050 mpu; // defining a sensor called mpu
SoftwareSerial ss(4,3); // defining a SoftwareSerial device called ss
TinyGPS gps;
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

float getgps(TinyGPS &gps);


const byte address[5] = {'U','r','m','o','m'};

// timekeeping and misc. vars
float currenttime; //used to be long but this caused timing issues where a decimal point was omitted
unsigned long lasttime;
int16_t ax, ay, az, gx, gy, gz;
int16_t gyro_angle_x_l, gyro_angle_y_l;
int16_t angle_x_l, angle_y_l;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
const float KMPH = 3.6;
const float ALPHA = 0.96;
const float CONST_16G = 2048;
const float CONST_2000 = 16.4;
const float CONST_G = 9.81;
const float RADIANS_TO_DEGREES = 180 / 3.14159;

unsigned long age;
float gpslat, gpslon, gpsalt, gpsspd;

const int sdcs = 5;

void setup(){
  // debugging
  // TODO: remove
  Serial.begin(115200);
  ss.begin(9600);    // gps

  // Set up oversampling and filter initialization
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Wire.begin();   // gyro; per MPU6050 lib, this joins i2c bus
  mpu.initialize();
  calibratesensors();  // initialize sensor hardware

  radio.begin();    // radio initialize
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(10,10);
  radio.openWritingPipe(address);
}
void loop(){
  bool sending;
  currenttime = millis(); // time from start (milliseconds)
  float alt = int(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  float pressure = bmp.pressure / 100.0;
  //getgps(gps); // lat lon alt spd
  bool sent; //checks if last packet was sent
  while(ss.available()){
    char c = ss.read();
      if(gps.encode(c)){
        gps.f_get_position(&gpslat, &gpslon,&age);
        gpsalt = gps.f_altitude();
        gpsspd = gps.f_speed_kmph();
      }
  }
  float radiodata[] = {
                        currenttime, // milliseconds
                        alt, // m
                        //pressure, //hpa
                        velocity(currenttime), // kmh
                        gpslat, gpslon,
                        (ax - ax_offset) / CONST_16G,
                        (ay - ay_offset) / CONST_16G,
                        (az - az_offset) / CONST_16G,
                      };
    sending = radio.write(&radiodata, sizeof(radiodata)); // simultaneously define if message was sent and sending message
    if(sending){
      Serial.print("Sent!\t");
    } else{
      Serial.print("Fail!\t");
    }
    sent = sending; // storing current loop radio status
    //TODO: delete this before flight
    for(int i = 0; i < 3; i++)
    {
      Serial.print(radiodata[i]);
      Serial.print('\t');
    }
    for(int i = 0; i<5;i++){
      Serial.print(radiodata[i+3],6);
      Serial.print('\t');
    }
    Serial.print(sizeof(radiodata));
    Serial.println();

  // end todo
  set_last_time(currenttime);
}

//funcs
void calibratesensors() {
  Serial.println("Setting offsets");
  int                   num_readings = 100;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;

  // Discard the first set of values read from the IMU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    x_accel += ax;
    y_accel += ay;
    z_accel += az;
    x_gyro += gx;
    y_gyro += gy;
    z_gyro += gz;
    delay(10);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  // Store the raw calibration values globally
  ax_offset = x_accel;
  ay_offset = y_accel;
  az_offset = z_accel;
  gx_offset = x_gyro;
  gy_offset = y_gyro;
  gz_offset = z_gyro;

  Serial.print("Offsets: ");
  Serial.print(ax_offset);
  Serial.print(", ");
  Serial.print(ay_offset);
  Serial.print(", ");
  Serial.print(az_offset);
  Serial.print(", ");
  Serial.print(gx_offset);
  Serial.print(", ");
  Serial.print(gy_offset);
  Serial.print(", ");
  Serial.println(gz_offset);
  
  Serial.println("Finishing Calibration");
}

float getgps(TinyGPS &gps){
  char c = ss.read();
  gps.encode(c);
  gps.f_get_position(&gpslat, &gpslon,&age); //get floating point lat/long
  gpsalt = gps.f_altitude();
  gpsspd = gps.f_speed_kmph();
  Serial.print(gpslat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : gpslat, 6);
  //return gpslat, gpslon, gpsalt, gpsspd;
}

// mathy stuff
float velocity(float t_now){
  float dt = get_dt(t_now);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_p = (ax - ax_offset) / CONST_16G;
  float ay_p = (ay - ay_offset) / CONST_16G;
  float az_p = (az - az_offset) / CONST_16G;

  float accel_angle_y = atan(-1 * ax_p / sqrt(pow(ay_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(ay_p / sqrt(pow(ax_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;

  float gx_p = (gx - gx_offset) / CONST_2000;
  float gy_p = (gy - gy_offset) / CONST_2000;
  float gz_p = (gz - gz_offset) / CONST_2000;

  float gyro_angle_x = gx_p * dt + get_last_angle_x();
  float gyro_angle_y = gy_p * dt + get_last_angle_y();

  float angle_x = ALPHA * gyro_angle_x + (1.0 - ALPHA) * accel_angle_x;
  float angle_y = ALPHA * gyro_angle_y + (1.0 - ALPHA) * accel_angle_y;

  float vel_x = (ax_p * (dt/1000) * CONST_G);
  float vel_y = (ay_p * (dt/1000) * CONST_G);
  float vel_z = (az_p * (dt/1000) * CONST_G);
  float vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2) + pow(vel_z, 2)) * KMPH;
  return vel;//, angle_x, angle_y;
  // sd.write(ax, ay, az, gx, gy, gz) or some function to write raw
}


inline unsigned long get_last_time() {
  return lasttime;
}

inline void set_last_time(unsigned long _time) {
  lasttime = _time;
}

inline float get_dt(unsigned long t_now) {
  return (t_now - get_last_time());
}
inline int16_t get_last_gyro_angle_x() {
  return gyro_angle_x_l;
}

inline void set_last_gyro_angle_x(int16_t _gyro_angle_x) {
  gyro_angle_x_l = _gyro_angle_x;
}

inline int16_t get_last_gyro_angle_y() {
  return gyro_angle_y_l;
}

inline void set_last_gyro_angle_y(int16_t _gyro_angle_y) {
  gyro_angle_y_l = _gyro_angle_y;
}

inline int16_t get_last_angle_x() {
  return angle_x_l;
}

inline void set_last_angle_x(int16_t _ang_x) {
  angle_x_l = _ang_x;
}

inline int16_t get_last_angle_y() {
  return angle_y_l;
}

inline void set_last_angle_y(int16_t _ang_y) {
  angle_y_l = _ang_y;
}


// TODO:check this shit
// write to sd
// can this write in csv?
void sdwrite(){

}

// notes
// some text
/*
from adafruit:
http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
this is the formula used to compute altitude
  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
*/

/*
Bibliography (spec. for modified code):
https://github.com/arashmarzi/speedometer/blob/master/speedometer.ino
https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123/2
https://aprs.gids.nl/nmea/
https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/
https://learnarduinonow.com/2014/02/22/arduino-ublox-neo-6m-gps.html
https://www.youtube.com/watch?v=IGusY74xMR8
*/