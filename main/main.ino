/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"


#define SerialDebug false  // TODO Set to true to get Serial output for debugging
#define READ_TIME 100 //constant read time of 10Hz

#define X_MAG_CORRECTION 470
#define Y_MAG_CORRECTION 120
#define Z_MAG_CORRECTION 125

#define Latitude 39.399501
#define Longitude -84.561335

MPU9250 Patella;
//MPU9250 Quad;


void TestSensor(MPU9250* sensor)
{
    sensor->MPU9250SelfTest(sensor->SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(sensor->SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(sensor->SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(sensor->SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(sensor->SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(sensor->SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(sensor->SelfTest[5],1); Serial.println("% of factory value");  
}

void ReadAccel(MPU9250* sensor)
{
	  sensor->readAccelData(sensor->accelCount);  // Read the x/y/z adc values
    sensor->getAres();
    
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    sensor->ax = (float)sensor->accelCount[0]*sensor->aRes;
    sensor->ay = (float)sensor->accelCount[1]*sensor->aRes;
    sensor->az = (float)sensor->accelCount[2]*sensor->aRes;
}

void ReadMag(MPU9250* sensor)
{
	sensor->readMagData(sensor->magCount);  // Read the x/y/z adc values
    sensor->getMres();
    
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    sensor->magbias[0] = X_MAG_CORRECTION;
    // User environmental x-axis correction in milliGauss TODO axis??
    sensor->magbias[1] = Y_MAG_CORRECTION;
    // User environmental x-axis correction in milliGauss
    sensor->magbias[2] = Z_MAG_CORRECTION;
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    // Get actual magnetometer value, this depends on scale being set
    sensor->mx = (float)sensor->magCount[0]*sensor->mRes*sensor->magCalibration[0] - sensor->magbias[0];
    sensor->my = (float)sensor->magCount[1]*sensor->mRes*sensor->magCalibration[1] - sensor->magbias[1];
    sensor->mz = (float)sensor->magCount[2]*sensor->mRes*sensor->magCalibration[2] - sensor->magbias[2];
}

void ReadGyro(MPU9250* sensor)
{
	  sensor->readGyroData(sensor->gyroCount);  // Read the x/y/z adc values
    sensor->getGres();
    //Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    sensor->gx = (float)sensor->gyroCount[0]*sensor->gRes;
    sensor->gy = (float)sensor->gyroCount[1]*sensor->gRes;
    sensor->gz = (float)sensor->gyroCount[2]*sensor->gRes;
}

void PrintOrientation(MPU9250* sensor)
{
    float* Q = getQ();
    
    //sensor->yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    //sensor->roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    sensor->pitch = -asin(2.0f * (Q[1] * Q[3] - Q[0] * Q[2]));
    sensor->pitch *= RAD_TO_DEG;
    //sensor->yaw   *= RAD_TO_DEG;
    //sensor->yaw   -= 8.5;
    //sensor->roll  *= RAD_TO_DEG;
    //Serial.print(sensor->yaw);
    //Serial.print(",");
    Serial.println(sensor->pitch);
    //Serial.print(",");
    //Serial.println(sensor->roll);

}

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(9600);
  SPI.setClockDivider(SPI_CLOCK_DIV64);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = Patella.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  //byte c_2 = Quad.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  //Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c_2, HEX);


  if (c == 0x71 /*&& c_2 == 0x71*/) // WHO_AM_I should always be 0x71
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    TestSensor(&Patella);
    //TestSensor(&Quad);

    // Calibrate gyro and accelerometers, load biases in bias registers
    Patella.calibrateMPU9250(Patella.gyroBias, Patella.accelBias);
    //Quad.calibrateMPU9250(Quad.gyroBias, Quad.accelBias);
    

    Patella.initMPU9250();
    //Quad.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = Patella.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    //byte d_2 = Quad.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    //Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d_2, HEX);
    //if(d_2 != 0x48 || d != 0x48){ Serial.println("magnetometer failed to calibrate"); return;}

    // Get magnetometer calibration from AK8963 ROM
    Patella.initAK8963(Patella.magCalibration);
    //Quad.initAK8963(Quad.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    delay(5000); // Loop forever if communication doesn't happen recursively
    setup();
  }
}

void loop()
{
    
	ReadAccel(&Patella);
	ReadGyro(&Patella);
	ReadMag(&Patella);    

  // Must be called before updating quaternions!
  Patella.updateTime();
  MahonyQuaternionUpdate(Patella.ax, Patella.ay, Patella.az, Patella.gx*DEG_TO_RAD,\
  	Patella.gy*DEG_TO_RAD, Patella.gz*DEG_TO_RAD, Patella.my, Patella.mx, Patella.mz, Patella.deltat);

    
  Patella.delt_t = millis() - Patella.count;
  //Quad.delt_t = millis() - Quad.count;
  
  if (Patella.delt_t > READ_TIME /*&& Quad.delt_t > READ_TIME */) 
  {
    PrintOrientation(&Patella);
    
    Patella.count = millis();
    Patella.sumCount = 0;
    Patella.sum = 0;
  }
}
