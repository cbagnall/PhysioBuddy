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
#define READ_TIME 100 //Send results to serial at time of 10Hz

#define Latitude 43.254738 //These are for 98 Ward 
#define Longitude -79.922589 
#define YAW_CORRECTION 9.13 //This was calculated from https://www.ngdc.noaa.gov/geomag-web/

#define X_MAG_CORRECTION_BIAS 51 //470
#define Y_MAG_CORRECTION_BIAS 333 //120
#define Z_MAG_CORRECTION_BIAS -51 //125
#define X_MAG_CORRECTION_SCALE 1.03
#define Y_MAG_CORRECTION_SCALE 1.06
#define Z_MAG_CORRECTION_SCALE 0.92

#define SerialSpeed 9600

//#define CALIBRATE_MAG

int global_timer_3_seconds = 0;
int global_average = 0;
#define DemoOffset 103
#define DEMO_SCALE 2

MPU9250 Patella(0x68);
MPU9250 Quad(0x69);

Quaternion Patella_orientation;
Quaternion Quad_orientation;

#ifdef CALIBRATE_MAG
void Calibrate_Mag_Bias(MPU9250* sensor) 
{
 uint16_t ii = 0, sample_count = 0;
 int32_t mag_bias[3] = {0, 0, 0};
 int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

 Serial.println("Mag Calibration: Wave device in a figure eight until done!");
 delay(4000);

  // shoot for ~fifteen seconds of mag data
  if(sensor->getMmode() == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  //if(sensor->getMmode() == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {
    sensor->readMagData(mag_temp);   // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) {mag_max[jj] = mag_temp[jj];}
      if(mag_temp[jj] < mag_min[jj]) {mag_min[jj] = mag_temp[jj];}
    }
    if(sensor->getMmode() == 0x02) {delay(135);} // at 8 Hz ODR, new mag data is available every 125 ms
  }

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  sensor->getMres();
  
  Serial.print("x bias: ");
  Serial.println(mag_bias[0]*sensor->mRes*sensor->magCalibration[0]);
  Serial.print("y bias: ");
  Serial.println(mag_bias[1]*sensor->mRes*sensor->magCalibration[1]);
  Serial.print("z bias: ");
  Serial.println(mag_bias[2]*sensor->mRes*sensor->magCalibration[2]);

  float temp[3] = {0,0,0};
  temp[0] = (mag_max[0] - mag_min[0])/2;
  temp[1] = (mag_max[1] - mag_min[1])/2;
  temp[2] = (mag_max[2] - mag_min[2])/2;
  float avg_rad = temp[0] + temp[1] + temp[2];
  avg_rad /= 3.0;
  
  Serial.print("x scale: ");
  Serial.println(avg_rad/temp[0]);
  Serial.print("y scale: ");
  Serial.println(avg_rad/temp[1]);
  Serial.print("z scale: ");
  Serial.println(avg_rad/temp[2]);
  
  Serial.println("Mag Calibration done!");
}
#endif /*Calibrate_Mag*/

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
    sensor->ax = (float)sensor->accelCount[0]*sensor->aRes;
    sensor->ay = (float)sensor->accelCount[1]*sensor->aRes;
    sensor->az = (float)sensor->accelCount[2]*sensor->aRes;
}

void ReadMag(MPU9250* sensor)
{
	  sensor->readMagData(sensor->magCount);  // Read the x/y/z adc values
    sensor->getMres();
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    // Get actual magnetometer value, this depends on scale being set
    sensor->mx = X_MAG_CORRECTION_SCALE*sensor->magCount[0]*sensor->mRes - X_MAG_CORRECTION_BIAS;/* (float)sensor->magCount[0]*sensor->mRes*sensor->magCalibration[0] - sensor->magbias[0];*/
    sensor->my = Y_MAG_CORRECTION_SCALE*sensor->magCount[1]*sensor->mRes - Y_MAG_CORRECTION_BIAS;/* (float)sensor->magCount[1]*sensor->mRes*sensor->magCalibration[1] - sensor->magbias[1];*/
    sensor->mz = Z_MAG_CORRECTION_SCALE*sensor->magCount[2]*sensor->mRes - Z_MAG_CORRECTION_BIAS;/* (float)sensor->magCount[2]*sensor->mRes*sensor->magCalibration[2] - sensor->magbias[2];*/
}

void ReadGyro(MPU9250* sensor)
{
	  sensor->readGyroData(sensor->gyroCount);  // Read the x/y/z adc values
    sensor->getGres();
    //Calculate the gyro value into actual degrees per second
    sensor->gx = (float)sensor->gyroCount[0]*sensor->gRes;
    sensor->gy = (float)sensor->gyroCount[1]*sensor->gRes;
    sensor->gz = (float)sensor->gyroCount[2]*sensor->gRes;
}

void PrintOrientation(MPU9250* sensor, Quaternion* orientation)
{
    const float* Q = orientation->getQ();
    
    //sensor->yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    //sensor->roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    sensor->pitch = -asin(2.0f * (Q[1] * Q[3] - Q[0] * Q[2]));
    sensor->pitch *= RAD_TO_DEG;
    //sensor->yaw   *= RAD_TO_DEG;
    //sensor->yaw   -= YAW_CORRECTION;
    //sensor->roll  *= RAD_TO_DEG;
    //Serial.print(sensor->yaw);
    //Serial.print(",");
    Serial.println(sensor->pitch);
    //Serial.print(",");
    //Serial.println(sensor->roll);

}

void Print_Difference(MPU9250* sensor1, Quaternion* orientation1, MPU9250* sensor2, Quaternion* orientation2, bool use_avg)
{
 const float* Q1 = orientation1->getQ(); 
 const float* Q2 = orientation2->getQ(); 

 //sensor1->yaw   = atan2(2.0f * (*(Q1+1) * *(Q1+2) + *Q1 * *(Q1+3)), *Q1 * *Q1 + *(Q1+1) * *(Q1+1) - *(Q1+2) * *(Q1+2) - *(Q1+3) * *(Q1+3));
 sensor1->roll  = atan2(2.0f * (*Q1 * *(Q1+1) + *(Q1+2) * *(Q1+3)), *Q1 * *Q1 - *(Q1+1) * *(Q1+1) - *(Q1+2) * *(Q1+2) + *(Q1+3) * *(Q1+3));
 //sensor1->pitch = -asin(2.0f * (Q1[1] * Q1[3] - Q1[0] * Q1[2]));
 //sensor1->pitch *= RAD_TO_DEG;
 //sensor1->yaw   *= RAD_TO_DEG;
 //sensor1->yaw   -= YAW_CORRECTION;
 sensor1->roll  *= RAD_TO_DEG * DEMO_SCALE;
 
 //Serial.print("Sensor 1: ");
 //Serial.print(sensor1->pitch);

 if(use_avg){
  Serial.print("Diff: ");
  Serial.println(sensor1->roll - global_average/global_timer_3_seconds + DemoOffset);
 }
 else
 {
  global_timer_3_seconds++;
  global_average += (sensor1->roll);
 }
 //Serial.print(", ");
 //Serial.println(sensor1->roll);
 //Serial.print(", Sensor 2: ");
 //Serial.print(", ");
 //sensor2->yaw   = atan2(2.0f * (*(Q2+1) * *(Q2+2) + *Q2 * *(Q2+3)), *Q2 * *Q2 + *(Q2+1) * *(Q2+1) - *(Q2+2) * *(Q2+2) - *(Q2+3) * *(Q2+3));
 //sensor2->roll  = atan2(2.0f * (*Q2 * *(Q2+1) + *(Q2+2) * *(Q2+3)), *Q2 * *Q2 - *(Q2+1) * *(Q2+1) - *(Q2+2) * *(Q2+2) + *(Q2+3) * *(Q2+3));
 //sensor2->pitch = -asin(2.0f * (Q2[1] * Q2[3] - Q2[0] * Q2[2]));
 //sensor2->pitch *= RAD_TO_DEG;
 //sensor2->yaw   *= RAD_TO_DEG;
 //sensor2->yaw   -= YAW_CORRECTION;
 //sensor2->roll  *= RAD_TO_DEG;
 //Serial.print(sensor2->pitch);
 //Serial.print(sensor2->yaw);
 //Serial.print(sensor2->yaw);
 //Serial.print(", ");
 //Serial.println(sensor1->yaw);
 //Serial.print(", Difference: ");
 //Serial.println(abs(sensor1->pitch - sensor2->pitch));  */
}


void setup()
{
  //--------------------------------------------------Initialize I2C at 400 kbit/sec, Serial to 9600 baud rate, and clock to 16Mhz / 64 = 250KHz
  Wire.begin();
  Serial.begin(SerialSpeed);

  delay(3000); //sleep for 3 seconds to give serial a chance to connect
  Serial.print("Enabled Serial: ");
  Serial.println(SerialSpeed);
  Serial.println("Waiting for bluetooth...");
  delay(5000); //sleep for 5 seconds to give bluetooth a chance to pair for demo
 
  

  // Read the WHO_AM_I register, this is a good test of communication  
  byte c = Patella.readByte(Patella.MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  byte c_2 = Quad.readByte(Quad.MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println("MPU9250 #1 is online...");

    // Start by performing self test and reporting values
    TestSensor(&Patella);

    // Calibrate gyro and accelerometers, load biases in bias registers
    Patella.calibrateMPU9250(Patella.gyroBias, Patella.accelBias);
    Patella.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = Patella.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if(d != 0x48){ Serial.println("magnetometer #1 failed to calibrate"); return;}

    // Get magnetometer calibration from AK8963 ROM
    Patella.initAK8963(Patella.magCalibration, Patella.magbias);
    
#ifdef CALIBRATE_MAG
  //------------------------------------- Debug to calculate offsets
  Calibrate_Mag_Bias(&Patella);
  delay(4000);
  exit(0);
  //------------------------------------
#endif
  } // if (c == 0x71)
  
  else
  {
    Serial.println("Could not connect to MPU9250 #1");
    delay(5000); // Loop forever if communication doesn't happen recursively
    setup();
  }

  if (c_2 == 0x71) // WHO_AM_I should always be 0x71&& c_2 == 0x71
  {
    Serial.println("MPU9250 #2 is online...");

    // Start by performing self test and reporting values
    TestSensor(&Quad);

    // Calibrate gyro and accelerometers, load biases in bias registers
    Quad.calibrateMPU9250(Quad.gyroBias, Quad.accelBias);    
    Quad.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d_2 = Quad.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if(d_2 != 0x48){ Serial.println("magnetometer #2 failed to calibrate..or #1, we're not sure.."); return;}

    // Get magnetometer calibration from AK8963 ROM
    Quad.initAK8963(Quad.magCalibration, Quad.magbias);    
  }
  
  else
  {
    Serial.println("Could not connect to MPU9250 #2");
    delay(5000); // Loop forever if communication doesn't happen recursively
    setup();
  } 
  Serial.println("Calibrating...");
} //setup()

void loop()
{
#ifdef CALIBRATE_MAG
  delay(4000);
  exit(0);  
#endif
  
	ReadAccel(&Patella);
	ReadGyro(&Patella);
	ReadMag(&Patella);   

  ReadAccel(&Quad);
  ReadGyro(&Quad);
  ReadMag(&Quad); 

  // Must be called before updating quaternions!
  Patella.updateTime();
  Quad.updateTime();
  
  Patella_orientation.MahonyQuaternionUpdate(Patella.ax, Patella.ay, Patella.az, Patella.gx*DEG_TO_RAD,\
  	Patella.gy*DEG_TO_RAD, Patella.gz*DEG_TO_RAD, Patella.my, Patella.mx, Patella.mz, Patella.deltat);
    
  Quad_orientation.MahonyQuaternionUpdate(Quad.ax, Quad.ay, Quad.az, Quad.gx*DEG_TO_RAD,\
    Quad.gy*DEG_TO_RAD, Quad.gz*DEG_TO_RAD, Quad.my, Quad.mx, Quad.mz, Quad.deltat);

    
  Patella.delt_t = millis() - Patella.count;
  Quad.delt_t = millis() - Quad.count;
  
  if (Patella.delt_t > READ_TIME && Quad.delt_t > READ_TIME ) 
  {
    //PrintOrientation(&Patella, &Patella_orientation);
    if( global_timer_3_seconds > (1000/READ_TIME) * 3) {
      Print_Difference(&Patella, &Patella_orientation, &Quad, &Quad_orientation, true);
    }
    else {
      Print_Difference(&Patella, &Patella_orientation, &Quad, &Quad_orientation, false);
    }
    
    Patella.count = millis();
    Patella.sumCount = 0;
    Patella.sum = 0;
    Quad.count = millis();
    Quad.sumCount = 0;
    Quad.sum = 0;
  }
}

