#include <Wire.h> 
#include "MPU6050.h"

boolean level_calibration_on = false;
boolean auto_level = true;

float pid_error = 0;

float Kp_roll = 2.5;
float Ki_roll = 0;
float Kd_roll = 18.75;
int roll_max = 400;
float Kp_pitch = Kp_roll;
float Ki_pitch = Ki_roll;
float Kd_pitch = Kd_roll;
int pitch_max = roll_max;
float Kp_yaw = 5;
float Ki_yaw = 0.02;
float Kd_yaw = 0;
int yaw_max = 400;

int8_t temp;
int16_t manual_takeoff_channel_3 = 0;
int16_t idle_speed = 0;

uint8_t led, start;
uint8_t error, error_counter, error_led;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle;
int16_t i;

// Channel 1 - Right horizontal - Roll
// Channel 2 - Right vertical - Pitch
// Channel 3 - Left verticcal - Throttle
// Channel 4 - Left horizontal - Yaw
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;

uint32_t error_timer;

int16_t acc_axis[4], gyro_axis[4], temperature;
int16_t cal_int;
int16_t manual_acc_pitch_cal = 0;
int16_t manual_acc_roll_cal = 0;
int16_t manual_gyro_roll_cal = 0;
int16_t manual_gyro_pitch_cal = 0;
int16_t manual_gyro_yaw_cal = 0;

int32_t gyro_axis_cal[4], acc_axis_cal[4];
bool manual_calibration = false;

float roll, pitch, yaw = 0;
float acc_pitch, acc_roll;

uint32_t loop_timer;
uint32_t last_rx_time = 0;
uint32_t acc_total;

float roll_level_adjust, pitch_level_adjust;
float I_roll, roll_setpoint, gyro_roll_in, pid_roll, prev_roll_error;
float I_pitch, pitch_setpoint, gyro_pitch_in, pid_pitch, prev_pitch_error; 
float I_yaw, yaw_setpoint, gyro_yaw_in, pid_yaw, prev_yaw_error;
float battery_voltage;

void setup() {
  pinMode(PA4, INPUT_ANALOG); // for battery voltage measurement
  
  Serial.begin(115200);     // high baud rate to reduce loop time since loop time should execute within 4ms
  Serial.println("start");
  Serial.println("Roll");
  Serial.print("Kp: ");
  Serial.println(Kp_roll);
  Serial.print("Ki: ");
  Serial.println(Ki_roll);
  Serial.print("Kd: ");
  Serial.println(Kd_roll);
  Serial.println("Pitch");
  Serial.print("Kp: ");
  Serial.println(Kp_pitch);
  Serial.print("Ki: ");
  Serial.println(Ki_pitch);
  Serial.print("Kd: ");
  Serial.println(Kd_pitch);  
  Serial.println("Yaw");
  Serial.print("Kp: ");
  Serial.println(Kp_yaw);
  Serial.print("Ki: ");
  Serial.println(Ki_yaw);
  Serial.print("Kd: ");
  Serial.println(Kd_yaw);
  // put your setup code here, to run once:
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  pinMode(PB3, INPUT);
  pinMode(PB4, INPUT);

  if(digitalRead(PB3) || digitalRead(PB4)) led=1;
  else led=0;

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);

  green_led(LOW);
  red_led(HIGH);

  timer_setup();
  delay(50);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  error = Wire.endTransmission();
  
  while (error != 0) {
    error = 2;
    error_signal();
    Serial.println("MPU not connected");
    delay(4);
  }
  gyro_setup();
  
  if(!manual_calibration) {
    for(i=0; i < 1250; i++) {
      if (i%125 == 0) digitalWrite(PB4, !digitalRead(PB4));
      delay(4);
    }
    i = 0;
  }
  
  calibrate_gyro();

  // manual_imu_calibration();
  // while(1);
  
  // when receiver is not connected any one of the channel will be low
  // So raise error signal and wait till receiver is connected
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990) {
    Serial.print(channel_1);
    Serial.print("\t");
    Serial.print(channel_2);
    Serial.print("\t");
    Serial.print(channel_3);
    Serial.print("\t");
    Serial.print(channel_4);
    Serial.print(" ");
    error = 3;
    error_signal();
    delay(4);
    Serial.println(F("Channel not connected"));
  }
  error = 0;

  // wait till throttle is in lower position
  while (channel_3 < 990 || channel_3 > 1050) {
    error = 4;
    error_signal();
    delay(4);
    Serial.println(F("Throttle not lowered"));
  }
  error = 0;

  // Setup completed turn off red led
  red_led(LOW);
  
  Serial.println("Setup Finish");
  temp=0;
  // ESC_setup();
  battery_voltage = (float)analogRead(4) / 112.81;
  loop_timer = micros();

  // Turn on green led to start
  green_led(HIGH);
  
}

void loop() {
  error_signal();
  read_gyro(); 

  gyro_roll_in = (gyro_roll_in * 0.7) + ((gyro_axis[1] / 65.5) * 0.3);  // in deg/s
  gyro_pitch_in = (gyro_pitch_in * 0.7) + ((gyro_axis[2] / 65.5) * 0.3);
  gyro_yaw_in = (gyro_yaw_in * 0.7) + ((gyro_axis[3] / 65.5) * 0.3);

  // Gyro angle calculations
  // Gyro give angular velocity in one second
  // but we need angular velocity in 4ms since we are calculating gyro in 250Hz
  // So multiply 0.004 with gyro reading
  // 500dps -> 65.5
  // 0.0000611 = 0.004/65.5
  pitch += gyro_axis[2] * 0.0000611;
  roll  += gyro_axis[1] * 0.0000611;
  yaw   += gyro_axis[3] * 0.0000611;
  



  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
  pitch -= roll * sin(gyro_axis[3] * 0.000001066);
  roll += pitch * sin(gyro_axis[3] * 0.000001066);

  //Accelerometer angle calculations
  if (acc_axis[1] > 4096)acc_axis[1] = 4096;                                          //Limit the maximum accelerometer value.
  if (acc_axis[1] < -4096)acc_axis[1] = -4096;                                        //Limit the maximum accelerometer value.
  if (acc_axis[2] > 4096)acc_axis[2] = 4096;                                          //Limit the maximum accelerometer value.
  if (acc_axis[2] < -4096)acc_axis[2] = -4096;                                        //Limit the maximum accelerometer value.

  // acc_total => 4096 (8g)
  // acc_total = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  // if (abs(acc_y) < acc_total) { 
  //   // use (float)acc_y because acc_y/acc_total is alwyas <1 (if int rounded off to 0)
  //   // 57.296=(180/3.14)
  //   acc_pitch = asin((float)acc_x/acc_total)*57.296; 
  // }
  // if (abs(acc_x) < acc_total) {
  //   acc_roll = asin((float)acc_y/acc_total)*57.296;
  // }

  acc_pitch = asin((float)acc_axis[1] / 4096) * 57.296;
  acc_roll = asin((float)acc_axis[2] / 4096) * 57.296;

  // to reduce drift of gyro small part of noisy acc is added to gyro
  roll = roll*0.996 + acc_roll*0.004;
  pitch = pitch*0.996 + acc_pitch*0.004;

  pitch_level_adjust = pitch * 15;
  roll_level_adjust = roll * 15;

  if (!auto_level) {
    pitch_level_adjust = 0;
    roll_level_adjust = 0;    
  } 

  // for starting throttle low and yaw left for step 1
  if (channel_3 < 1050 && channel_4 < 1050) {
    start = 1;
    Serial.println(F("Start 1"));
  }

  // after step 1 center the yaw to start motors for step 2
  if(start == 1 && channel_3 < 1050 && channel_4 > 1450) {
    start = 2;
    Serial.println(F("Start 2"));
    green_led(LOW);
    pitch = acc_pitch;
    roll = acc_roll;

    I_roll = 0;
    prev_roll_error = 0;
    I_pitch = 0;
    prev_pitch_error = 0;
    I_yaw = 0;
    prev_yaw_error = 0;
  }

  // to stop motor throlle low and yaw right
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;
    green_led(HIGH);
  }

  roll_setpoint = 0; 
  roll_setpoint -= roll_level_adjust;
  // Divide the channel pulse by 3 to get the degree of rotation
  // In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s )
  roll_setpoint /= 3.0;               

  pitch_setpoint = 0;
  pitch_setpoint -= pitch_level_adjust;
  // Divide the channel pulse by 3 to get the degree of rotation
  // In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s )
  pitch_setpoint /= 3.0;              

  yaw_setpoint = 0;

  calculate_pid();

  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);
  if (battery_voltage < 10.0 && error == 0){
    error = 1;
    Serial.println("Battery Low");
  }
  throttle = channel_3;
  transmit_data();

  if (start == 2) {
    if (throttle > 1800) throttle = 1800;                     //We need some room to keep full control at full throttle.
    
    esc_1 = throttle - pid_pitch - pid_roll + pid_yaw;        //Calculate the pulse for esc 1 (front-left - CW).
    esc_2 = throttle - pid_pitch + pid_roll - pid_yaw;        //Calculate the pulse for esc 2 (front-right - CCW).
    esc_3 = throttle + pid_pitch - pid_roll - pid_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle + pid_pitch + pid_roll + pid_yaw;        //Calculate the pulse for esc 4 (rear-right - CW).

    // esc_1 = throttle - pid_pitch + pid_roll - pid_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    // esc_2 = throttle + pid_pitch + pid_roll + pid_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    // esc_3 = throttle + pid_pitch - pid_roll - pid_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    // esc_4 = throttle - pid_pitch - pid_roll + pid_yaw;        //Calculate the pulse for esc 4 (front-left - CW).



    if (esc_1 < 1100) esc_1 = 1000;                           
    if (esc_2 < 1100) esc_2 = 1000;                           
    if (esc_3 < 1100) esc_3 = 1000;                           
    if (esc_4 < 1100) esc_4 = 1000;                           

    if (esc_1 > 2000) esc_1 = 2000;                              //Limit the esc-1 pulse to 1500us.
    if (esc_2 > 2000) esc_2 = 2000;                              //Limit the esc-2 pulse to 1500us.
    if (esc_3 > 2000) esc_3 = 2000;                              //Limit the esc-3 pulse to 1500us.
    if (esc_4 > 2000) esc_4 = 2000;                              //Limit the esc-4 pulse to 1500us.
  }
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  TIMER3_BASE->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER3_BASE->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER3_BASE->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER3_BASE->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.

  if (micros()-loop_timer > 4050) {
    Serial.println("loop time exceeded");
    error = 5;
  }
  
    Serial.print(roll, 2);
  Serial.print("\t");
  Serial.print(pitch, 2);
  Serial.print("\t");
  Serial.print(yaw, 2);
  Serial.println("\t"); 
  while(micros()-loop_timer < 4000);      // loop timer is set to previous loop timing (curr_time-prev_time<4000us=)
  loop_timer = micros();       // micros() gives time in us
}
