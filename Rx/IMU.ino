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

  if (cal_int == 0) {
    gyro_axis_cal[1] = 0;
    gyro_axis_cal[2] = 0;
    gyro_axis_cal[3] = 0;
    Serial.println("Calibrating the gyro...");
    for(cal_int=0; cal_int < 2000; cal_int++) {
    //  if (cal_int%25 == 0) digitalWrite(PB4, !digitalRead(PB4));  
      read_gyro();
      gyro_axis_cal[1] += gyro_axis[1];         // as gyro gives angular velocity (rad/s) integrate to get angular position
      gyro_axis_cal[2] += gyro_axis[2];
      gyro_axis_cal[3] += gyro_axis[3];
      delay(4);
    }
    gyro_axis_cal[1] /= 2000;
    gyro_axis_cal[2] /= 2000;
    gyro_axis_cal[3] /= 2000;
    manual_gyro_roll_cal = gyro_axis_cal[1];
    manual_gyro_pitch_cal = gyro_axis_cal[2];    
    manual_gyro_yaw_cal = gyro_axis_cal[3];
  }
}

//This part reads the raw gyro and accelerometer data from the MPU-6050
void read_gyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 14);

  acc_axis[1] = Wire.read() << 8 | Wire.read();
  acc_axis[2] = Wire.read() << 8 | Wire.read();
  acc_axis[3] = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_axis[1] = Wire.read() << 8 | Wire.read();
  gyro_axis[2] = Wire.read() << 8 | Wire.read();
  gyro_axis[3] = Wire.read() << 8 | Wire.read();
  
  gyro_axis[2] *= -1;
  gyro_axis[3] *= -1;
  
  acc_axis[1] -= manual_acc_pitch_cal;
  acc_axis[2] -= manual_acc_roll_cal;
  gyro_axis[1] -= manual_gyro_roll_cal;
  gyro_axis[2] -= manual_gyro_pitch_cal;
  gyro_axis[3] -= manual_gyro_yaw_cal;
}

//In this part the various registers of the MPU-6050 are set.
void gyro_setup() {
  Wire.setClock(200000); // Set the clock speed of I2C to 400KHz
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

void manual_imu_calibration() {
  gyro_axis_cal[1] = 0;                                                             //Reset calibration variables for next calibration.
  gyro_axis_cal[2] = 0;                                                             //Reset calibration variables for next calibration.
  gyro_axis_cal[3] = 0; 
  acc_axis_cal[1] = 0; 
  acc_axis_cal[2] = 0; 
  Serial.print("Calibrating gyro");
  for(cal_int = 0; cal_int < 4000; cal_int++) {
    read_gyro();
    acc_axis_cal[1] += acc_axis[1];
    acc_axis_cal[2] += acc_axis[2];
    gyro_axis_cal[1] += gyro_axis[1];                                              
    gyro_axis_cal[2] += gyro_axis[2];                                              
    gyro_axis_cal[3] += gyro_axis[3];  
    Serial.print(cal_int);
    Serial.print(". ");
    Serial.print(acc_axis_cal[1]); 
    Serial.print(" ");
    Serial.print(acc_axis_cal[2]);
    Serial.print(" ");
    Serial.println(gyro_axis_cal[2]);
    delay(4);
  }
  acc_axis_cal[1] /= 4000;
  acc_axis_cal[2] /= 4000;
  gyro_axis_cal[1] /= 4000;
  gyro_axis_cal[2] /= 4000;
  gyro_axis_cal[3] /= 4000;
  Serial.print("manual_acc_pitch_cal_value = ");
  Serial.println(acc_axis_cal[1]);
  Serial.print("manual_acc_roll_cal_value = ");
  Serial.println(acc_axis_cal[2]);
  Serial.print("manual_gyro_pitch_cal_value = ");
  Serial.println(gyro_axis_cal[1]);
  Serial.print("manual_gyro_roll_cal_value = ");
  Serial.println(gyro_axis_cal[2]);
  Serial.print("manual_gyro_yaw_cal_value = ");
  Serial.println(gyro_axis_cal[3]);
}

