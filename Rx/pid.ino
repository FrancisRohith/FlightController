void calculate_pid(void) {
  // Roll PID
  pid_error = gyro_roll_in - roll_setpoint;

  I_roll += Ki_roll * pid_error;
  if(I_roll > roll_max) I_roll = roll_max;
  else if (I_roll < roll_max * -1) I_roll = roll_max * -1;

  pid_roll = Kp_roll * pid_error + I_roll + Kd_roll * (pid_error - prev_roll_error);
  if (pid_roll > roll_max) pid_roll = roll_max;
  else if (pid_roll < roll_max * -1) pid_roll = roll_max * -1;

  prev_roll_error = pid_error;

  // Pitch PID
  pid_error = gyro_pitch_in - pitch_setpoint;
  
  I_pitch += Ki_pitch * pid_error;
  if (I_pitch > pitch_max) I_pitch = pitch_max;
  else if (I_pitch < pitch_max * -1) I_pitch = pitch_max * -1;

  pid_pitch = Kp_pitch * pid_error + I_pitch + Kd_pitch * (pid_error - prev_pitch_error);

  if (pid_pitch > pitch_max) pid_pitch = pitch_max;
  else if (pid_pitch < pitch_max * -1) pid_pitch = pitch_max * -1;

  prev_pitch_error = pid_error;

  // Yaw PID
  pid_error = gyro_yaw_in - yaw_setpoint;
  I_yaw += Ki_yaw * pid_error;
  if (I_yaw > yaw_max) I_yaw = yaw_max;
  else if (I_yaw < yaw_max * -1) I_yaw = yaw_max * -1;

  pid_yaw = Kp_yaw * pid_error + I_yaw + Kd_yaw * (pid_error - prev_yaw_error);
  if (pid_yaw > yaw_max) pid_yaw = yaw_max;
  else if (pid_yaw < yaw_max * -1) pid_yaw = yaw_max * -1;

  prev_yaw_error = pid_error; 
}