void pid(void) {
  pitch_setpoint = 0;
  roll_setpoint = 0;
  yaw_setpoint = 0;

  // Roll PID
  error = roll_gyro - roll_setpoint;
  I_roll = Igain_roll * error;
  if(I_roll > roll_max) I_roll = roll_max;
  else if (I_roll < roll_max * -1) I_roll = roll_max * -1;

  pid_roll = Pgain_roll * error + I_roll + Dgain_roll * (error-prev_error);
  if (pid_roll > roll_max) pid_roll = roll_max;

  error = pitch_gyro - pitch_setpoint;
  pitch_I = pitch_I_gain * error;
  if (pid)
}