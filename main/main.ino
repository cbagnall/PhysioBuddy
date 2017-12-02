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

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
#define SleepTime 100

MPU9250 Patella;
MPU9250 Quad;

//Store sensitivity values
float ini_x_pat;
float ini_y_pat;
float ini_z_pat;
float ini_x_quad;
float ini_y_quad;
float ini_z_quad;

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

void SetMagCalibrationValues(MPU9250* sensor, float* x, float* y, float* z)
{
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(sensor->magCalibration[0], 2);
      *x = sensor->magCalibration[0];
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(sensor->magCalibration[1], 2);
      *y = sensor->magCalibration[1];
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(sensor->magCalibration[2], 2);
      *z = sensor->magCalibration[2];	
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
  byte c_2 = Quad.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c_2, HEX);


  if (c == 0x71 && c_2 == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    TestSensor(&Patella);
    TestSensor(&Quad);

    // Calibrate gyro and accelerometers, load biases in bias registers
    Patella.calibrateMPU9250(Patella.gyroBias, Patella.accelBias);
    Quad.calibrateMPU9250(Quad.gyroBias, Quad.accelBias);
    

    Patella.initMPU9250();
    Quad.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = Patella.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    byte d_2 = Quad.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d_2, HEX);
    if(d_2 != 0x48 || d != 0x48){ Serial.println("magnetometer failed to calibrate"); return;}

    // Get magnetometer calibration from AK8963 ROM
    Patella.initAK8963(Patella.magCalibration);
    Quad.initAK8963(Quad.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
		SetMagCalibrationValues(&Patella, &ini_x_pat, &ini_y_pat, &ini_z_pat);
		SetMagCalibrationValues(&Quad, &ini_x_quad, &ini_y_quad, &ini_z_quad);
    }

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
    Patella.readAccelData(Patella.accelCount);  // Read the x/y/z adc values
    Patella.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    Patella.ax = (float)Patella.accelCount[0]*Patella.aRes; // - accelBias[0];
    Patella.ay = (float)Patella.accelCount[1]*Patella.aRes; // - accelBias[1];
    Patella.az = (float)Patella.accelCount[2]*Patella.aRes; // - accelBias[2];

    Patella.readGyroData(Patella.gyroCount);  // Read the x/y/z adc values
    Patella.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    Patella.gx = (float)Patella.gyroCount[0]*Patella.gRes;
    Patella.gy = (float)Patella.gyroCount[1]*Patella.gRes;
    Patella.gz = (float)Patella.gyroCount[2]*Patella.gRes;

    Patella.readMagData(Patella.magCount);  // Read the x/y/z adc values
    Patella.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    Patella.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    Patella.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    Patella.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    Patella.mx = (float)Patella.magCount[0]*Patella.mRes*Patella.magCalibration[0] -
               Patella.magbias[0];
    Patella.my = (float)Patella.magCount[1]*Patella.mRes*Patella.magCalibration[1] -
               Patella.magbias[1];
    Patella.mz = (float)Patella.magCount[2]*Patella.mRes*Patella.magCalibration[2] -
               Patella.magbias[2];

  // Must be called before updating quaternions!
  Patella.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(Patella.ax, Patella.ay, Patella.az, Patella.gx*DEG_TO_RAD,
                         Patella.gy*DEG_TO_RAD, Patella.gz*DEG_TO_RAD, Patella.my,
                         Patella.mx, Patella.mz, Patella.deltat);


Patella.delt_t = millis() - Patella.count;
if (Patella.delt_t > SleepTime)
{
  if(SerialDebug)
  {
    // Print acceleration values in milligs!
//        Serial.print("X-acceleration: "); Serial.print(1000*Patella.ax);
//        Serial.println(" mg ");
//        Serial.print("Y-acceleration: "); Serial.print(1000*Patella.ay);
//        Serial.print(" mg ");
//        Serial.print("Z-acceleration: "); Serial.print(1000*Patella.az);
//        Serial.println(" mg ");
    float acc_x = 905/8*Patella.ax;
    float acc_y = 100*Patella.ay;
    float acc_z = 100*Patella.az;
    float new_x = acc_x/ini_x_pat;
    float new_x2 = 0.0074*new_x*new_x + 0.7876*new_x + 3.0508;
    
    Serial.print(new_x + new_x2);
    Serial.print(",");
    Serial.print(new_x);
    Serial.print(",");
    Serial.print(acc_y/ini_y_pat);
    Serial.print(",");
    Serial.println(acc_z/ini_z_pat);
    
//        // Print gyro values in degree/sec
    //Serial.print("X-gyro rate: "); Serial.print(Patella.gx, 3);
    //Serial.println(" degrees/sec ");
//        Serial.print("Y-gyro rate: "); Serial.print(Patella.gy, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Z-gyro rate: "); Serial.print(Patella.gz, 3);
//        Serial.println(" degrees/sec");
//
//        // Print mag values in degree/sec
    //Serial.print("X-mag field: "); Serial.print(Patella.mx);
    //Serial.print(" mG ");
//        Serial.print("Y-mag field: "); Serial.print(Patella.my);
//        Serial.print(" mG ");
//        Serial.print("Z-mag field: "); Serial.print(Patella.mz);
//        Serial.println(" mG");
#if 0
    Patella.tempCount = Patella.readTempData();  // Read the adc values
    // Temperature in degrees Centigrade
    Patella.temperature = ((float) Patella.tempCount) / 333.87 + 21.0;
    // Print temperature in degrees Centigrade
#endif
//        Serial.print("Temperature is ");  Serial.print(Patella.temperature, 1);
//        Serial.println(" degrees C");
  }

  Patella.count = millis();
} // if (Patella.delt_t > 500)

  #if 0
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    Patella.delt_t = millis() - Patella.count;

    // update LCD once per half-second independent of read rate
    if (Patella.delt_t > SleepTime)
    {
      if(SerialDebug)
      {
        Serial.print("ax = "); Serial.print((int)1000*Patella.ax);
        Serial.print(" ay = "); Serial.print((int)1000*Patella.ay);
        Serial.print(" az = "); Serial.print((int)1000*Patella.az);
        Serial.println(" mg");

        Serial.print("gx = "); Serial.print( Patella.gx, 2);
        Serial.print(" gy = "); Serial.print( Patella.gy, 2);
        Serial.print(" gz = "); Serial.print( Patella.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = "); Serial.print( (int)Patella.mx );
        Serial.print(" my = "); Serial.print( (int)Patella.my );
        Serial.print(" mz = "); Serial.print( (int)Patella.mz );
        Serial.println(" mG");

        Serial.print("q0 = "); Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      Patella.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      Patella.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      Patella.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      Patella.pitch *= RAD_TO_DEG;
      Patella.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      Patella.yaw   -= 8.5;
      Patella.roll  *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(Patella.yaw, 2);
        Serial.print(", ");
        Serial.print(Patella.pitch, 2);
        Serial.print(", ");
        Serial.println(Patella.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)Patella.sumCount/Patella.sum, 2);
        Serial.println(" Hz");
      }
      Patella.count = millis();
      Patella.sumCount = 0;
      Patella.sum = 0;
    } // if (Patella.delt_t > 500)
  } // if (AHRS)
#endif
  delay(SleepTime);
}
