//MY NOTES
//Cannot use pin 13 from MPU example as it is one of the SPI pins!
//Needed to do a lot of changes to get SD.open to work see:
//https://forum.arduino.cc/t/sd-card-does-not-open-file/212389
#include <SD.h>
#include <SPI.h>
File myFile;
int pinCS = 10; // Pin 10 on Arduino Uno
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at
https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2024-07-04 - Copied code base onto github SpeedyWhegs3
// 2019-07-08 - Added Auto Calibration and offset generator
// - and altered FIFO retrieval sequence to avoid using blocking code
// 2016-04-18 - Eliminated a potential infinite loop
// 2013-05-08 - added seamless Fastwire support
// - added note about gyro calibration
// 2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
// 2012-06-20 - improved FIFO overflow handling and simplified read process
// 2012-06-19 - completely rearranged DMP initialization code and simplification
// 2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
// 2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
// 2012-06-05 - add gravity-compensated initial reference frame acceleration output
// - add 3D math helper file to DMP6 example sketch
// - add Euler output and Yaw/Pitch/Roll output formats
// 2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
// 2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
// 2012-05-30 - basic DMP initialization working
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN
THE SOFTWARE.
===============================================
*/
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE
implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation
board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
/*
============================================================
=============
NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
depends on the MPU-6050's INT pin being connected to the Arduino's
external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
digital I/O pin 2.
============================================================
============= */
/*
============================================================
=============
NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
when using Serial.write(buf, len). The Teapot output uses this method.
The solution requires a modification to the Arduino USBAPI.h file, which
is fortunately simple, but annoying. This will be fixed in the next IDE
release. For more info, see these links:
http://arduino.cc/forum/index.php/topic,109987.0.html
http://code.google.com/p/arduino/issues/detail?id=958
============================================================
============= */
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the
actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION
// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER
// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the
yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL
// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see
acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL
// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6) //See top note
//bool blinkState = false;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity
vector
// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
mpuInterrupt = true;
}
// ================================================================
// === INITIAL SETUP ===
// ================================================================
void setup() {
//===== SD Card Section
============================================================
==========================//
//================================================================
============================================//
pinMode(pinCS, OUTPUT);
if (SD.begin()){
Serial.println("SD card is ready to use.");
} else{
Serial.println("SD card initialization failed");
return;
}
//======MPU6050
Section======================================================
=================================//
//================================================================
============================================//
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
compilation difficulties
// https://github.com/jrowberg/i2cdevlib/issues/519#issuecomment-
752023021
// Vasko added this
Wire.setWireTimeout(3000, true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif
// initialize serial communication
// (115200 chosen because it is required for Teapot Demo output, but it's
// really up to you depending on your project)
Serial.begin(115200);
while (!Serial); // wait for Leonardo enumeration, others continue immediately
// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
// Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
// the baud timing being too misaligned with processor ticks. You must use
// 38400 or slower in these cases, or use some kind of external separate
// crystal solution for the UART timer.
// initialize device
//Serial.println(F("Initializing I2C devices..."));
mpu.initialize();
pinMode(INTERRUPT_PIN, INPUT);
// verify connection
//Serial.println(F("Testing device connections..."));
//Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
// wait for ready
Serial.println(F("\nSend any character to begin DMP programming and demo:
"));
while (Serial.available() && Serial.read()); // empty buffer
while (!Serial.available()); // wait for data
while (Serial.available() && Serial.read()); // empty buffer again
// load and configure the DMP
//Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize();
// supply your own gyro offsets here, scaled for min sensitivity
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
// make sure it worked (returns 0 if so)
if (devStatus == 0) {
// Calibration Time: generate offsets and calibrate our MPU6050
mpu.CalibrateAccel(6);
mpu.CalibrateGyro(6);
mpu.PrintActiveOffsets();
// turn on the DMP, now that it's ready
//Serial.println(F("Enabling DMP..."));
mpu.setDMPEnabled(true);
// enable Arduino interrupt detection
//Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//Serial.println(F(")..."));
attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady,
RISING);
mpuIntStatus = mpu.getIntStatus();
// set our DMP Ready flag so the main loop() function knows it's okay to use it
//Serial.println(F("DMP ready! Waiting for first interrupt...")); dmpReady = true;
// get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize();
} else {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
// configure LED for output
// pinMode(LED_PIN, OUTPUT); //See top note
}
// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================
void loop() {
// if programming failed, don't try to do anything
if (!dmpReady) return;
// read a packet from FIFO
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_QUATERNION
// display quaternion values in easy matrix form: w x y z
mpu.dmpGetQuaternion(&q, fifoBuffer);
Serial.print(F("quat\t"));
Serial.print(F(q.w));
Serial.print(F("\t"));
Serial.print(F(q.x));
Serial.print(F("\t"));
Serial.print(F(q.y));
Serial.print(F("\t"));
Serial.println(F(q.z));
#endif
#ifdef OUTPUT_READABLE_EULER
// display Euler angles in degrees
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetEuler(euler, &q);
Serial.print(F("euler\t"));
Serial.print(F(euler[0] * 180 / M_PI));
Serial.print(F("\t"));
Serial.print(F(euler[1] * 180 / M_PI));
Serial.print(F("\t"));
Serial.println(F(euler[2] * 180 / M_PI));
#endif
#ifdef OUTPUT_READABLE_YAWPITCHROLL
// display Euler angles in degrees
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
Serial.print("ypr\t");
Serial.print(ypr[0] * 180 / M_PI);
Serial.print("\t");
Serial.print(ypr[1] * 180 / M_PI);
Serial.print("\t");
//Serial.println(ypr[2] * 180/M_PI); //I COMMENTED THIS ONE TO GET
BOTH GYRO AND ACC
Serial.print(ypr[2] * 180 / M_PI);
Serial.print("\t");
#endif
#ifdef OUTPUT_READABLE_REALACCEL
// display real acceleration, adjusted to remove gravity
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetAccel(&aa, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
Serial.print("areal\t");
Serial.print(aaReal.x);
Serial.print("\t");
Serial.print(aaReal.y);
Serial.print("\t");
Serial.println(aaReal.z);
#endif
#ifdef OUTPUT_READABLE_WORLDACCEL
// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetAccel(&aa, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
Serial.print(F("aworld\t"));
Serial.print(F(aaWorld.x));
Serial.print(F("\t"));
Serial.print(F(aaWorld.y));
Serial.print(F("\t"));
Serial.println(F(aaWorld.z));
#endif
#ifdef OUTPUT_TEAPOT
// display quaternion values in InvenSense Teapot demo format:
teapotPacket[2] = fifoBuffer[0];
teapotPacket[3] = fifoBuffer[1];
teapotPacket[4] = fifoBuffer[4];
teapotPacket[5] = fifoBuffer[5];
teapotPacket[6] = fifoBuffer[8];
teapotPacket[7] = fifoBuffer[9];
teapotPacket[8] = fifoBuffer[12];
teapotPacket[9] = fifoBuffer[13];
Serial.write(teapotPacket, 14);
teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
// blink LED to indicate activity
// blinkState = !blinkState;
// digitalWrite(LED_PIN, blinkState); //See top note
}
//===== SD Card Section
============================================================
==========================//
//==========================================================
==================================================//
myFile = SD.open("test.txt", FILE_WRITE);
// if the file opened okay, write to it:
if (myFile) {
// Write to file
//myFile.print("Testing text 1, 2 ,3...");
myFile.print(String(micros()));
myFile.print("\t");
myFile.print("ypr\t");
myFile.print(ypr[0] * 180 / M_PI);
myFile.print("\t");
myFile.print(String(ypr[1] * 180 / M_PI));
myFile.print("\t");
myFile.print(String(ypr[2] * 180 / M_PI));
myFile.print("\t");
myFile.print("areal\t");
myFile.print(String(aaReal.x));
myFile.print("\t");
myFile.print(String(aaReal.y));
myFile.print("\t");
myFile.println(String(aaReal.z));
myFile.close(); // close the file
Serial.println("Done.");
}
// if the file didn't open, print an error:
else {
Serial.println("error opening test.txt");
}
}
3.2 IMU Mobile Robot Version
//MY NOTES
//Cannot use pin 13 from MPU example as it is one of the SPI pins!
//Needed to do a lot of changes to get SD.open to work see:
//https://forum.arduino.cc/t/sd-card-does-not-open-file/212389
#include <SD.h>
#include <SPI.h>
File myFile;
int pinCS = 10; // Pin 10 on Arduino Uno
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at
https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2019-07-08 - Added Auto Calibration and offset generator
// - and altered FIFO retrieval sequence to avoid using blocking
code
// 2016-04-18 - Eliminated a potential infinite loop
// 2013-05-08 - added seamless Fastwire support
// - added note about gyro calibration
// 2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility
error
// 2012-06-20 - improved FIFO overflow handling and simplified read process
// 2012-06-19 - completely rearranged DMP initialization code and
simplification
// 2012-06-13 - pull gyro and accel data from FIFO packet instead of reading
directly
// 2012-06-09 - fix broken FIFO read sequence and change interrupt detection
to RISING
// 2012-06-05 - add gravity-compensated initial reference frame acceleration output
// - add 3D math helper file to DMP6 example sketch
// - add Euler output and Yaw/Pitch/Roll output formats
// 2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
// 2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
// 2012-05-30 - basic DMP initialization working
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN
THE SOFTWARE.
===============================================
*/
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE
implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation
board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
/*
============================================================
=============
NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
depends on the MPU-6050's INT pin being connected to the Arduino's
external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
digital I/O pin 2.
============================================================
============= */
/*
============================================================
=============
NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
when using Serial.write(buf, len). The Teapot output uses this method.
The solution requires a modification to the Arduino USBAPI.h file, which
is fortunately simple, but annoying. This will be fixed in the next IDE
release. For more info, see these links:
http://arduino.cc/forum/index.php/topic,109987.0.html
http://code.google.com/p/arduino/issues/detail?id=958
============================================================
============= */
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the
actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION
// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER
// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the
yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL
// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see
acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL
// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6) //See top note
//bool blinkState = false;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity
vector
// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
mpuInterrupt = true;
}
// ================================================================
// === INITIAL SETUP ===
// ================================================================
void setup() {
//===== SD Card Section
============================================================
==========================//
//================================================================
============================================//
pinMode(pinCS, OUTPUT);
SD.begin();
// if (SD.begin()){
// Serial.println("SD card is ready to use.");
// } else{
// Serial.println("SD card initialization failed");
// return;
// }
//======MPU6050
Section======================================================
=================================//
//================================================================
============================================//
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
compilation difficulties
// https://github.com/jrowberg/i2cdevlib/issues/519#issuecomment-752023021
// Vasko added this
Wire.setWireTimeout(3000, true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif
// initialize serial communication
// (115200 chosen because it is required for Teapot Demo output, but it's
// really up to you depending on your project)
// Serial.begin(115200); //If you want to debug
// while (!Serial); // wait for Leonardo enumeration, others continue
immediately
// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
// Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
// the baud timing being too misaligned with processor ticks. You must use
// 38400 or slower in these cases, or use some kind of external separate
// crystal solution for the UART timer.
// initialize device
//Serial.println(F("Initializing I2C devices..."));
mpu.initialize();
pinMode(INTERRUPT_PIN, INPUT);
// verify connection
//Serial.println(F("Testing device connections..."));
//Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
// wait for ready
// Serial.println(F("\nSend any character to begin DMP programming and demo:"));
// while (Serial.available() && Serial.read()); // empty buffer
// while (!Serial.available()); // wait for data
// while (Serial.available() && Serial.read()); // empty buffer again
// load and configure the DMP
//Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize();
// supply your own gyro offsets here, scaled for min sensitivity
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
// make sure it worked (returns 0 if so)
if (devStatus == 0) {
// Calibration Time: generate offsets and calibrate our MPU6050
mpu.CalibrateAccel(6);
mpu.CalibrateGyro(6);
mpu.PrintActiveOffsets();
// turn on the DMP, now that it's ready
//Serial.println(F("Enabling DMP..."));
mpu.setDMPEnabled(true);
// enable Arduino interrupt detection
//Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//Serial.println(F(")..."));
attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady,
RISING);
mpuIntStatus = mpu.getIntStatus();
// set our DMP Ready flag so the main loop() function knows it's okay to use it
//Serial.println(F("DMP ready! Waiting for first interrupt..."));
dmpReady = true;
// get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize();
} else {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
// Serial.print(F("DMP Initialization failed (code "));
// Serial.print(devStatus);
// Serial.println(F(")"));
}
// configure LED for output
// pinMode(LED_PIN, OUTPUT); //See top note
}
// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================
void loop() {
// if programming failed, don't try to do anything
if (!dmpReady) return;
// read a packet from FIFO
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_QUATERNION
// display quaternion values in easy matrix form: w x y z
mpu.dmpGetQuaternion(&q, fifoBuffer);
// Serial.print(F("quat\t"));
// Serial.print(F(q.w));
// Serial.print(F("\t"));
// Serial.print(F(q.x));
// Serial.print(F("\t"));
// Serial.print(F(q.y));
// Serial.print(F("\t"));
// Serial.println(F(q.z));
#endif
#ifdef OUTPUT_READABLE_EULER
// display Euler angles in degrees
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetEuler(euler, &q);
// Serial.print(F("euler\t"));
// Serial.print(F(euler[0] * 180 / M_PI));
// Serial.print(F("\t"));
// Serial.print(F(euler[1] * 180 / M_PI));
// Serial.print(F("\t"));
// Serial.println(F(euler[2] * 180 / M_PI));
#endif
#ifdef OUTPUT_READABLE_YAWPITCHROLL
// display Euler angles in degrees
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
// Serial.print("ypr\t");
// Serial.print(ypr[0] * 180 / M_PI);
// Serial.print("\t");
// Serial.print(ypr[1] * 180 / M_PI);
// Serial.print("\t");
// //Serial.println(ypr[2] * 180/M_PI); //I COMMENTED THIS ONE TO GET
BOTH GYRO AND ACC
// Serial.print(ypr[2] * 180 / M_PI);
// Serial.print("\t");
#endif
#ifdef OUTPUT_READABLE_REALACCEL
// display real acceleration, adjusted to remove gravity
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetAccel(&aa, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
// Serial.print("areal\t");
// Serial.print(aaReal.x);
// Serial.print("\t");
// Serial.print(aaReal.y);
// Serial.print("\t");
// Serial.println(aaReal.z);
#endif
#ifdef OUTPUT_READABLE_WORLDACCEL
// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetAccel(&aa, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
// Serial.print(F("aworld\t"));
// Serial.print(F(aaWorld.x));
// Serial.print(F("\t"));
// Serial.print(F(aaWorld.y));
// Serial.print(F("\t"));
// Serial.println(F(aaWorld.z));
#endif
#ifdef OUTPUT_TEAPOT
// display quaternion values in InvenSense Teapot demo format:
teapotPacket[2] = fifoBuffer[0];
teapotPacket[3] = fifoBuffer[1];
teapotPacket[4] = fifoBuffer[4];
teapotPacket[5] = fifoBuffer[5];
teapotPacket[6] = fifoBuffer[8];
teapotPacket[7] = fifoBuffer[9];
teapotPacket[8] = fifoBuffer[12];
teapotPacket[9] = fifoBuffer[13];
// Serial.write(teapotPacket, 14);
teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
// blink LED to indicate activity
// blinkState = !blinkState;
// digitalWrite(LED_PIN, blinkState); //See top note
}
//===== SD Card Section
============================================================
==========================//
//==========================================================
==================================================//
myFile = SD.open("test.txt", FILE_WRITE);
// if the file opened okay, write to it:
if (myFile) {
// Write to file
//myFile.print("Testing text 1, 2 ,3...");
myFile.print(String(micros()));
myFile.print("\t");
myFile.print("ypr\t");
myFile.print(ypr[0] * 180 / M_PI);
myFile.print("\t");
myFile.print(String(ypr[1] * 180 / M_PI));
myFile.print("\t");
myFile.print(String(ypr[2] * 180 / M_PI));
myFile.print("\t");
myFile.print("areal\t");
myFile.print(String(aaReal.x));
myFile.print("\t");
myFile.print(String(aaReal.y));
myFile.print("\t");
myFile.println(String(aaReal.z));
myFile.close(); // close the file
Serial.println("Done.");
}
// // if the file didn't open, print an error:
// else {
// Serial.println("error opening test.txt");
// }
}
3.3 Arduino ODrive PWM for Double Rack Steering
#include <Servo.h>
//Pin 10 to ODriveGPIO 4 to axis1
//Pin 9 to ODriveGPIO 3 to axis0
int FrServPin = 11;
int BaServPin = 3;
int ax0Pin = 9;
int ax1Pin = 10;
//Create servo objects
Servo FrServ;
Servo BaServ;
Servo ax0;
Servo ax1;
//Here's where we'll keep our channel values
int ch1;
int ch1r;
int ch2;
int ch3;
int ch4;
int ch5;
void setup() {
//Attach servo objects to pins
FrServ.attach(FrServPin);
FrServ.writeMicroseconds(1500);
BaServ.attach(BaServPin);
BaServ.writeMicroseconds(1500);
ax0.attach(ax0Pin);
ax0.writeMicroseconds(1000);
ax1.attach(ax1Pin);
ax1.writeMicroseconds(1000);
//Reciever input pins
pinMode(2, INPUT);
pinMode(4, INPUT);
pinMode(5, INPUT);
pinMode(6, INPUT);
pinMode(7, INPUT);
//If monitoring, otherwise comment out
Serial.begin(9600);
}
void loop() {
ch1 = pulseIn(2, HIGH, 50000); // Read the pulse width of
ch2 = pulseIn(4, HIGH, 50000);
ch3 = pulseIn(5, HIGH, 50000);
ch4 = pulseIn(6, HIGH, 50000);
ch5 = pulseIn(7, HIGH, 50000);
//Spec sheet limits are 800 to 2200, but 700 to 2300 is also possible
ch1 = map(ch1, 900, 2100, 800, 2200);
ch1r= map(ch1, 800, 2200, 2200, 800);
//Standard 50Hz pwm limits are 1000 to 2000
ch2 = map(ch2, 1550, 2062, 1000, 2000);
//Removing Jitter from servo steering
if (ch1 > 1460 & ch1 < 1590) {
ch1 = 1525;}
FrServ.writeMicroseconds(ch1);
BaServ.writeMicroseconds(ch1r);
ax0.writeMicroseconds(ch2);
ax1.writeMicroseconds(ch2);
Serial.print("Steering:");
Serial.print(ch1);
Serial.print("\t Throttle:");
Serial.println(ch2);
}
3.4 Arduino ODrive PWM for Skid Steering
#include <Servo.h>
//Pin 10 to ODriveGPIO 4 to axis1
//Pin 9 to ODriveGPIO 3 to axis0
int FrServPin = 11;
int BaServPin = 3;
int ax0Pin = 9;
int ax1Pin = 10;
//Create servo objects
Servo FrServ;
Servo BaServ;
Servo ax0;
Servo ax1;
//Here's where we'll keep our channel values
int ch1;
int ch1r;
int ch2;
int ch3;
int ch4;
int ch5;
float num0;
float num1;
int sp0;
int sp1;
void setup() {
//Attach servo objects to pins
FrServ.attach(FrServPin);
FrServ.writeMicroseconds(1500);
BaServ.attach(BaServPin);
BaServ.writeMicroseconds(1500);
ax0.attach(ax0Pin);
ax0.writeMicroseconds(1000);
ax1.attach(ax1Pin);
ax1.writeMicroseconds(1000);
//Reciever input pins
pinMode(2, INPUT);
pinMode(4, INPUT);
pinMode(5, INPUT);
pinMode(6, INPUT);
pinMode(7, INPUT);
//If monitoring, otherwise comment out
Serial.begin(9600);
FrServ.writeMicroseconds(1525);
BaServ.writeMicroseconds(1525);
}
void loop() {
ch1 = pulseIn(2, HIGH, 50000); // Read the pulse width of
ch2 = pulseIn(4, HIGH, 50000);
ch3 = pulseIn(5, HIGH, 50000);
ch4 = pulseIn(6, HIGH, 50000);
ch5 = pulseIn(7, HIGH, 50000);
//Spec sheet limits are 800 to 2200, but 700 to 2300 is also possible
ch1 = map(ch1, 800, 2200, 50, 150);
ch1r = map(ch1, 50, 150, 150, 50);
//Standard 50Hz pwm limits are 1000 to 2000
//666*150/2+1000 for max speed
ch2 = map(ch2, 1550, 2062, 0, 666);
//Removing Jitter from servo steering
// if (ch1 > 1460 & ch1 < 1590) {
// ch1 = 1525;}
num0=(float)ch1/100;
num1=(float)ch1r/100;
sp0=ch2*(num0);
sp1=ch2*(num1);
ax0.writeMicroseconds(sp0+1000);
ax1.writeMicroseconds(sp1+1000);
Serial.print("ch1:");
Serial.print(ch1);
Serial.print("\t num0:");
Serial.print(num0);
Serial.print("\t sp0:");
Serial.print(sp0+1000);
Serial.print("\t\t ch1r:");
Serial.print(ch1r);
Serial.print("\t num1:");
Serial.print(num1);
Serial.print("\t sp1:");
Serial.println(sp1+1000);
}
