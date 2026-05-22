/*
PIN CONNECTION
VCC_IN - 3v3
SCL - PB10
SDA - PB11
*/ 
////////////ABOUT ACCELEROMETER AND GYRO///////////
// Accelerometer are less sensitive to drift but highly sensitive to vibration
// Gyro is less sensitive to vibration but drifts more by time
///////////////////////////////////////////////

// This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
void calibrate_gyro() {
  // Even if gyro is not moving output seems to be fluctuating so take 
  // average of the readings and subract while reading gyro
  if(manual_calibration) cal_int = 2000;
  else {
    cal_int = 0;
    manual_gyro_pitch_cal = 0;
    manual_gyro_roll_cal = 0;
    manual_gyro_yaw_cal = 0;
  }

  if (cal_int != 2000) {
    cal_gyro_roll = 0;
    cal_gyro_pitch = 0;
    cal_gyro_yaw = 0;
    Serial.println("Calibrating the gyro...");
    for(cal_int=0; cal_int < 2000; cal_int++) {
    //  if (cal_int%25 == 0) digitalWrite(PB4, !digitalRead(PB4));  
      read_gyro();
      cal_gyro_roll += gyro_roll;         // as gyro gives angular velocity (rad/s) integrate to get angular position
      cal_gyro_pitch += gyro_pitch;
      cal_gyro_yaw += gyro_yaw;
      delay(4);
    }
    cal_gyro_roll /= 2000;
    cal_gyro_pitch /= 2000;
    cal_gyro_yaw /= 2000;
    manual_gyro_roll_cal = cal_gyro_roll;
    manual_gyro_pitch_cal = cal_gyro_pitch;
    manual_gyro_yaw_cal = cal_gyro_yaw;
  }
}

//This part reads the raw gyro and accelerometer data from the MPU-6050
void read_gyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_roll = Wire.read() << 8 | Wire.read();
  gyro_pitch = Wire.read() << 8 | Wire.read();
  gyro_yaw = Wire.read() << 8 | Wire.read();
  gyro_pitch *= -1;
  gyro_yaw *= -1;
  
  acc_x -= manual_acc_pitch_cal;
  acc_y -= manual_acc_roll_cal;
  gyro_roll -= manual_gyro_roll_cal;
  gyro_pitch -= manual_gyro_pitch_cal;
  gyro_yaw -= manual_gyro_yaw_cal;
}

//In this part the various registers of the MPU-6050 are set.
void gyro_setup() {
  Wire.setClock(400000); // Set the clock speed of I2C to 400KHz
  Wire.beginTransmission(MPU_ADDR);   // (0x68 << 1) + W/R -> 0/1
  Wire.write(PWR_MGMT_1); // target register address is PWR_MNGT_1
  Wire.write(0x00);       // reset all bit to start device and continue in power mode
  Wire.endTransmission(true); 

  Wire.beginTransmission(0x68); // MPU6050
  Wire.write(CONFIG);      
  Wire.write(0x03);             // Set DLPF to 43Hz
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);           // ACCEL_CONFIG register
  Wire.write(0x10);           // ±8g range
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);     // GYRO_CONFIG register
  Wire.write(0x08);     // ±500dps
  Wire.endTransmission(true);
}

void print_angle() {
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.println(yaw);
}
