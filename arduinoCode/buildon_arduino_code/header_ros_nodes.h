#ifndef HEADER_ROS_H
#define HEADER_ROS_H

ros::NodeHandle nh;


std_msgs::Int16 arduino_hearbeat_msg;
ros::Publisher arduinoHeartbeat("arduino_heartbeat", &arduino_hearbeat_msg);

std_msgs::Float32 debug1_msg;
ros::Publisher debug1("debug1", &debug1_msg);
std_msgs::Float32 debug2_msg;
ros::Publisher debug2("debug2", &debug2_msg);


std_msgs::Int16 pwm_right_msg;
ros::Publisher pub_pwm_right("pwm_right", &pwm_right_msg);

std_msgs::Int16 pwm_left_msg;
ros::Publisher pub_pwm_left("pwm_left", &pwm_left_msg);


std_msgs::Int64 pub_ticks_left_msg;
ros::Publisher pub_ticks_left("pub_ticks_left", &pub_ticks_left_msg);

std_msgs::Int64 pub_ticks_right_msg;
ros::Publisher pub_ticks_right("pub_ticks_right", &pub_ticks_right_msg);


std_msgs::Float32 left_wheel_vel_msg;
ros::Publisher left_wheel_vel("/left_wheel_vel", &left_wheel_vel_msg);

std_msgs::Float32 right_wheel_vel_msg;
ros::Publisher right_wheel_vel("/right_wheel_vel", &right_wheel_vel_msg);


std_msgs::Float32 desired_left_wheel_vel_msg;
ros::Publisher desired_left_wheel_vel("/desired_left_wheel_vel", &left_wheel_vel_msg);

std_msgs::Float32 desired_right_wheel_vel_msg;
ros::Publisher desired_right_wheel_vel("/desired_right_wheel_vel", &desired_right_wheel_vel_msg);


std_msgs::Float32 pub_control_part_p_msg;
ros::Publisher pub_control_part_p("/pub_control_part_p", &pub_control_part_p_msg);

std_msgs::Float32 pub_control_part_i_msg;
ros::Publisher pub_control_part_i("/pub_control_part_i", &pub_control_part_i_msg);

std_msgs::Float32 pub_control_part_d_msg;
ros::Publisher pub_control_part_d("/pub_control_part_d", &pub_control_part_d_msg);


void setControlLoopTime( const std_msgs::Float32& msg){
  control_loop_time = msg.data;
  }
ros::Subscriber <std_msgs::Float32> set_control_loop_time("set_control_loop_time", setControlLoopTime);

void kpValue( const std_msgs::Float32& msg){
  Kp = msg.data;
  }
ros::Subscriber <std_msgs::Float32> Kp_value("kp_value", kpValue);

void kiValue( const std_msgs::Float32& msg){
  Ki = msg.data;
  }
ros::Subscriber <std_msgs::Float32> Ki_value("ki_value", kiValue);

void kdValue( const std_msgs::Float32& msg){
  Kd = msg.data;
}
ros::Subscriber <std_msgs::Float32> Kd_value("kd_value", kdValue);

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(5, HIGH-digitalRead(5));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

//Create a message callback that updates the motor speeds
void cmdVelLinear(const std_msgs::Float32& cmd_vel)
{
    last_cmd_vel_time = millis();
    linear_vel = cmd_vel.data; 
}
ros::Subscriber <std_msgs::Float32> cmd_vel_linear("/cmd_vel_linear", &cmdVelLinear);
void cmdVelAngular(const std_msgs::Float32& cmd_vel)
{
    last_cmd_vel_time = millis();
    angular_vel = cmd_vel.data; 

}
ros::Subscriber <std_msgs::Float32> cmd_vel_angular("/cmd_vel_angular", &cmdVelAngular);



void cmdSetTn(const std_msgs::Float32& cmdSetTn)
{
    Tn = cmdSetTn.data; 
    Ki = Kp / Tn;
    i_error_left = 0;
    i_error_right = 0;
}
ros::Subscriber <std_msgs::Float32> setTn("/set_Tn", &cmdSetTn);



void cmdSetTv(const std_msgs::Float32& cmdSetTv)
{
    Tv = cmdSetTv.data; 
    Kd = Kp * Tv;
}
ros::Subscriber <std_msgs::Float32> setTv("/set_Tv", &cmdSetTv);
#endif
