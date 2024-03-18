// RECEIVES CMD_VEL FROM ROS

bool doesnt_go_straight = false;
void get_desired_tick_velocity () {
  // calculates single wheel velocities

  if (millis() - last_cmd_vel_time > 1000) linear_vel = 0;
  if (millis() - last_cmd_vel_time > 1000) angular_vel = 0;

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

 if (angular_vel != 0) {
  doesnt_go_straight = true;
  }

if (doesnt_go_straight && angular_vel == 0) {
  left_offset = pulseCount_l;
  right_offset = pulseCount_r;  
  doesnt_go_straight = false;
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

  measuredVelRunningAverageLeft.addValue(delta_pulseCount_l * 1000 / delta_pulseTime);
  measuredVelRunningAverageRight.addValue(delta_pulseCount_r * 1000 / delta_pulseTime);
  measured_vel_ticks_right = measuredVelRunningAverageRight.getAverage();
  measured_vel_ticks_left = measuredVelRunningAverageLeft.getAverage();

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


//float right_offset_last = 0;
float desired_delta_ticks = 0;
void delta_position_control () {

  float tick_left = pulseCount_l - left_offset;
  
  float tick_right = pulseCount_r - right_offset;  
  
  delta_ticks = tick_right - tick_left;
  
  float adapted_vel_ticks = Kp_pos * (desired_delta_ticks - delta_ticks);


  if (angular_vel == 0) {
    adapted_vel_ticks_left = desired_vel_ticks_left - adapted_vel_ticks;
    adapted_vel_ticks_right = desired_vel_ticks_right + adapted_vel_ticks;    
    }
  else {
    adapted_vel_ticks_left = desired_vel_ticks_left;
    adapted_vel_ticks_right = desired_vel_ticks_right;    
    }
  }


unsigned long pid_previous_time = 0;

float last_p_error_left = 0;
float last_p_error_right = 0;

int max_pwm = 50;

void calc_pwm_output () {
  int elapsed_time = 50;
  // calculate P part
  int p_error_left = adapted_vel_ticks_left - measured_vel_ticks_left;
  int p_error_right = adapted_vel_ticks_right - measured_vel_ticks_right;

  // calculate I part
  // checks if i part stays within the pwm range. otherwise it dosnt keep adding up
  if ((p_error_left > 0 && i_part_left <= max_pwm) || (p_error_left < 0 && i_part_left >= -1*max_pwm)) {
  i_error_left += elapsed_time * p_error_left;  
  }
  if ((p_error_right > 0 && i_part_right <= max_pwm) || (p_error_right < 0 && i_part_right >= -1*max_pwm)) {
  i_error_right += elapsed_time * p_error_right;  
  }

  
  // calculate D part
  float d_error_left = (p_error_left - last_p_error_left) / elapsed_time;
  float d_error_right = (p_error_right - last_p_error_right) / elapsed_time;

  // set last P error for next loop
  last_p_error_left = p_error_left;
  last_p_error_right = p_error_right;

  // calculate PWM output
  p_part_left = Kp * p_error_left;
  p_part_right = Kp * p_error_right;

  i_part_left = Ki * i_error_left;
  i_part_right = Ki * i_error_right;

  d_part_left = Kd * d_error_left;
  d_part_right = Kd * d_error_right;

  pwm_left = Kr * desired_vel_ticks_left + p_part_left + i_part_left + d_part_left;
  
  pwm_right = Kr * desired_vel_ticks_right + p_part_right + i_part_right + d_part_right;
  

  if (pwm_right < 0 && !currently_right_wheel_forwards) pwm_right = - 1 * pwm_right;
  if (pwm_right > max_pwm) pwm_right = max_pwm;
  else if (pwm_right < 0) pwm_right = 0;

  

  if (pwm_left < 0 && !currently_left_wheel_forwards) pwm_left = - 1 * pwm_left;
  if (pwm_left > max_pwm) pwm_left = max_pwm;
  else if (pwm_left < 0) pwm_left = 0;


}


void send_pwm_signal() {
  // set left PWM
  analogWrite(Speed_l, pwm_left); //Speed

  // set right PWM
  analogWrite(Speed_r, pwm_right); //Speed
}
