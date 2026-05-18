#include <Wire.h> 
#include "MPU6050.h"
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

bool level_calibration_on = false;
bool set_gyro = false;

float Kp_roll = 0;
float Ki_roll = 0;
float Kd_roll = 0;
int roll_max = 400;

float Kp_pitch = Kp_roll;
float Ki_pitch = Ki_roll;
float kd_pitch = Kd_roll;
int pitch_max = roll_max;

float Kp_yaw = 0;
float Ki_yaw = 0;
float Kd_yaw = 0;
int yaw_max = 400;

int16_t manual_takeoff_channel_3 = 0;
int16_t idle_speed = 0;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle;

int16_t acc_x, acc_y, acc_z;
int16_t temperature;
int16_t cal_int;
int16_t manual_acc_pitch_cal = 0;
int16_t manual_acc_roll_cal = 0;
int16_t manual_gyro_roll_cal = 0;
int16_t manual_gyro_pitch_cal = 0;
int16_t manual_gyro_yaw_cal = 0;

int32_t cal_gyro_roll = 0;
int32_t cal_gyro_pitch = 0;
int32_t cal_gyro_yaw = 0;
bool manual_calibration = false;

float roll, pitch, yaw = 0;
int16_t gyro_roll, gyro_pitch, gyro_yaw;  // dont use float
float acc_pitch, acc_roll;

uint32_t loop_timer;
uint32_t last_rx_time = 0;
uint32_t acc_total;

float roll_level_adjust, pitch_level_adjust;
float error;
float I_roll, roll_setpoint, gyro_roll_in, pid_roll, prev_roll_error;
float I_pitch, pitch_setpoint, gyro_pitch_in, pid_pitch, prev_pitch_error; 
float I_yaw, yaw_setpoint, gyro_yaw_in, pid_yaw, prev_yaw_error;

const byte addressA[6] = "00001";
const byte addressB[6] = "00011";

struct Transmitter {
  float roll;
  float pitch;
  float yaw;
};

struct Receiver {
  uint16_t channel_3;
};

Transmitter Tx;
Receiver Rx;
RF24 radio(PB0, PA4); // PB0 = CE, PA4 = CSN

void setup() {
  // put your setup code here, to run once:
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  Serial.begin(115200);     // high baud rate to reduce loop time since loop time should execute within 4ms
  ESC_setup();
  delay(50);
  gyro_setup();
  nRF_setup();
  if(!manual_calibration) {
    delay(5000);
  }
  
  calibrate_gyro();
  loop_timer = micros();
}

void loop() {
  read_gyro();  
  
  gyro_roll_in = (gyro_roll_in * 0.7) + (((float)gyro_roll / 65.5) * 0.3);  // in deg/s
  gyro_pitch_in = (gyro_pitch_in * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
  gyro_yaw_in = (gyro_yaw_in * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);
  // Gyro angle calculations
  // Gyro give angular velocity in one second
  // but we need angular velocity in 4ms since we are calculating gyro in 250Hz
  // So multiply 0.004 with gyro reading
  // 500dps -> 65.5
  // 0.0000611 = 0.004/65.5
  pitch += (float)gyro_pitch * 0.0000611;
  roll  += (float)gyro_roll * 0.0000611;
  yaw   += (float)gyro_yaw * 0.0000611;
  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
  pitch -= roll * sin((float)gyro_yaw * 0.000001066);
  roll += pitch * sin((float)gyro_yaw * 0.000001066);
  
  acc_total = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  if (abs(acc_y) < acc_total) {
    // use (float)acc_y because acc_y/acc_total is alwyas <1 (if int rounded off to 0)
    // 57.296=(180/3.14)
    acc_pitch = asin((float)acc_x/acc_total)*57.296; 
    acc_pitch -= -0.6;
  }
  if (abs(acc_x) < acc_total) {
    acc_roll = asin((float)acc_y/acc_total)*57.296;
    acc_roll -= -0.4;
  }

  if (set_gyro) {
    // to reduce drift small part of noisy acc is added to gyro
    roll = roll*0.9996 + acc_roll*0.0004;
    pitch = pitch*0.9996 + acc_pitch*0.0004;
  } else {
    // Initially if MPU is inclined it cant be detected by gyro so use
    // acclerometer for initial angle;
    roll=acc_roll;
    pitch=acc_pitch;
    I_pitch = 0;
    prev_pitch_error = 0;
    I_roll = 0;
    prev_roll_error = 0;
    I_yaw = 0;
    prev_yaw_error = 0;
    set_gyro=true;
  }

  pitch_level_adjust = pitch * 15;
  roll_level_adjust = roll * 15;

  roll_setpoint = 0;
  roll_setpoint -= roll_level_adjust;
  roll_setpoint /= 3.0;

  pitch_setpoint = 0;
  pitch_setpoint -= pitch_level_adjust;
  pitch_setpoint /= 3.0;

  yaw_setpoint = 0;

  Tx.roll = roll;
  Tx.pitch = pitch;
  Tx.yaw = yaw;
  if(radio.available()) {    
    digitalWrite(PC13, LOW);
    while (radio.available()) {
      radio.read(&Rx, sizeof(Rx));\
      throttle = Rx.channel_3;
      last_rx_time = micros();
    }
    radio.stopListening();
    radio.write(&Tx, sizeof(Tx));    
    radio.startListening();
  } 

  if(Serial.available()) {
    char c = Serial.read();
    if(c == 't') transmit_data();
  }

  pid();

  if (throttle > 1300) throttle = 1300;                                          //We need some room to keep full control at full throttle.
  esc_1 = throttle - pid_pitch + pid_roll - pid_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
  esc_2 = throttle + pid_pitch + pid_roll + pid_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
  esc_3 = throttle + pid_pitch - pid_roll - pid_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
  esc_4 = throttle - pid_pitch - pid_roll + pid_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

  if (esc_1 < 1000) esc_1 = 1000;                                                //Keep the motors running.
  if (esc_2 < 1000) esc_2 = 1000;                                                //Keep the motors running.
  if (esc_3 < 1000) esc_3 = 1000;                                                //Keep the motors running.
  if (esc_4 < 1000) esc_4 = 1000;                                                //Keep the motors running.

  if (esc_1 > 1500)esc_1 = 1500;                                                 //Limit the esc-1 pulse to 1500us.
  if (esc_2 > 1500)esc_2 = 1500;                                                 //Limit the esc-2 pulse to 1500us.
  if (esc_3 > 1500)esc_3 = 1500;                                                 //Limit the esc-3 pulse to 1500us.
  if (esc_4 > 1500)esc_4 = 1500;                                                 //Limit the esc-4 pulse to 1500us.

  TIMER2_BASE->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER2_BASE->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER2_BASE->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER2_BASE->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.

  if (micros() - last_rx_time > 500000) {
    Serial.println("last_rx_time exceeded");
    throttle = 1000;
    digitalWrite(PC13, HIGH);
    TIMER2_BASE->CCR1 = 1000;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
    TIMER2_BASE->CCR2 = 1000;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
    TIMER2_BASE->CCR3 = 1000;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
    TIMER2_BASE->CCR4 = 1000;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
  }  
//  print_angle();
  if (micros()-loop_timer > 4050){ 
    Serial.println("Loop time exceeded ");
  }
  while(micros()-loop_timer < 4000);      // loop timer is set to previous loop timing (curr_time-prev_time<4000us)
  loop_timer = micros();       // micros() gives time in us

}
