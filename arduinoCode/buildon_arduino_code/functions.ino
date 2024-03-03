// RECEIVES CMD_VEL FROM ROS
void get_desired_tick_velocity () {
  // calculates single wheel velocities
  desired_vel_meters_right = linear_vel + (angular_vel * base_width / 2);
  desired_vel_meters_left = linear_vel + (angular_vel * (-1) * base_width / 2);

  // calculates desired ticks per second
  desired_vel_ticks_right = desired_vel_meters_right * ticks_per_meter;  // goal for ticks/second
  desired_vel_ticks_left = desired_vel_meters_left * ticks_per_meter;  // goal for ticks/second

  if (desired_vel_ticks_right >= 0) desired_right_wheel_forwards = true;
  else {
    desired_right_wheel_forwards = false;
    //desired_vel_ticks_right = -1 * desired_vel_ticks_right;
  }

  if (desired_vel_ticks_left >= 0) desired_left_wheel_forwards = true;
  else {
    desired_left_wheel_forwards = false;
    //desired_vel_ticks_left = -1 * desired_vel_ticks_left;
  }
}

float pulseCount_l_last = 0;
float pulseCount_r_last = 0;
unsigned long pulseTime_last = 0; // last time encoder count got taken

void get_measured_tick_velocity() {
  // set values
  unsigned long pulseTime_now = millis();
  float pulseCount_r_now = pulseCount_r;
  float pulseCount_l_now = pulseCount_l;

  float delta_pulseTime = pulseTime_now - pulseTime_last;

  float delta_pulseCount_r = pulseCount_r_now - pulseCount_r_last;
  float delta_pulseCount_l = pulseCount_l_now - pulseCount_l_last;


  // calc tick velocity
  measured_vel_ticks_right = delta_pulseCount_r * 1000 / delta_pulseTime;
  measured_vel_ticks_left = delta_pulseCount_l * 1000 / delta_pulseTime;
/*
  if (!currently_right_wheel_forwards) measured_vel_ticks_right = -1 * measured_vel_ticks_right;
  if (!currently_left_wheel_forwards) measured_vel_ticks_left = -1 * measured_vel_ticks_left;*/
  
  // sets values for next iteration
  pulseCount_r_last = pulseCount_r_now;
  pulseCount_l_last = pulseCount_l_now;
  pulseTime_last = pulseTime_now;


}

void publish_measured_velocity () {
  measured_vel_meters_right = measured_vel_ticks_right * one_tick_distance;
  measured_vel_meters_left = measured_vel_ticks_left * one_tick_distance;

  right_wheel_vel_msg. data = measured_vel_meters_right;
  right_wheel_vel.publish(&right_wheel_vel_msg);

  left_wheel_vel_msg. data = measured_vel_meters_left;
  left_wheel_vel.publish(&left_wheel_vel_msg);
  
  desired_left_wheel_vel_msg.data = desired_vel_meters_left;
  desired_left_wheel_vel.publish(&desired_left_wheel_vel_msg);
  
  desired_right_wheel_vel_msg.data = desired_vel_meters_right;
  desired_right_wheel_vel.publish(&desired_right_wheel_vel_msg);
}



unsigned long pid_previous_time = 0;
float i_error_left = 0;
float i_error_right = 0;
float last_p_error_left = 0;
float last_p_error_right = 0;
int pwm_left = 0;
int pwm_right = 0;


void calc_pwm_output () {
  unsigned long current_time = millis();
  int elapsed_time = current_time - pid_previous_time;

  // calculate P part
  int p_error_left = desired_vel_ticks_left - measured_vel_ticks_left;
  int p_error_right = desired_vel_ticks_right - measured_vel_ticks_right;

  // calculate I part
  i_error_left += elapsed_time * p_error_left;
  i_error_right += elapsed_time * p_error_right;

  // calculate D part
  float d_error_left = (p_error_left - last_p_error_left) / elapsed_time;
  float d_error_right = (p_error_right - last_p_error_right) / elapsed_time;

  // set last P error for next loop
  last_p_error_left = p_error_left;
  last_p_error_right = p_error_right;

  // calculate PWM output


  debug1_msg.data= desired_vel_ticks_left;
  debug1.publish(&debug1_msg);
  debug2_msg.data= measured_vel_ticks_left;
  debug2.publish(&debug2_msg);
  
  pwm_left = Kp * p_error_left + Ki * i_error_left + Kd * d_error_left;  
  pwm_right = Kp * p_error_right + Ki * i_error_right + Kd * d_error_right;

  
  if (pwm_right > 255) pwm_right = 255;
  if (pwm_right < 0 && !currently_right_wheel_forwards) pwm_right = - 1 * pwm_right;
  else if (pwm_right < 0) pwm_right = 0;
  pwm_right_msg.data = pwm_right;
  pub_pwm_right.publish(&pwm_right_msg);

  
  if (pwm_left > 255) pwm_left = 255;
  if (pwm_left < 0 && !currently_left_wheel_forwards) pwm_left = - 1 * pwm_left;
  else if (pwm_left < 0) pwm_left = 0;
  pwm_left_msg.data = pwm_left;
  pub_pwm_left.publish(&pwm_left_msg);
}


void send_pwm_signal() {
  // set left PWM
  analogWrite(Speed_l, pwm_left); //Speed

  // set right PWM
  analogWrite(Speed_r, pwm_right); //Speed
}
