#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include "RunningAverage.h"

int n = 0; // for debugging

int Speed_l= 13; //VR 0-255
int Direction_l= 12; // HIGH for forwards. !HIGH for backwards
int Encoder_l=2;
int Enable_l= 11; // EL Start=High, Stop=Low 


//right Wheel PINS
int Enable_r= 4; // EL Start=High, Stop=Low 
int Encoder_r= 3; 
int Direction_r= 5; // !HIGH for forwards. HIGH for backwards
int Speed_r= 9;  //VR 0-255


// robot parameters
const float base_width = 0.37; // base width of roboter
const float wheel_diameter = 0.16; 
const float wheel_circumference = PI * wheel_diameter;
const int ticks_per_circ = 45;
const float one_tick_distance = wheel_circumference / ticks_per_circ;
const float ticks_per_meter = 1 / one_tick_distance;


// defines frequency of control loop

unsigned long last_loop_time = 0;

int ros_com_loop_time = 100;
unsigned long last_ros_com_loop_time = 0;

unsigned long last_cmd_vel_time = - 1100; // sets time of last cmd_vel publish
unsigned long loop_time_1 = 0; // for debugging 
///////////////////// PID control values

int control_loop_time = 50;
float Kp = 0.625;

float Kp_pos = 1;
float adapted_vel_ticks_left;
float adapted_vel_ticks_right;
int left_offset = 0;
int right_offset = 0;

int pwm_left = 0;
int pwm_right = 0;

int delta_ticks;

float Tn = 1800;
float Ki =  Kp / Tn;
float Kr = 1.4444;
float Tv = 300;
float Kd = Kp * Tv;
float i_error_left = 0;
float i_error_right = 0;

float p_part_left = 0;
float p_part_right = 0;

float i_part_left = 0;
float i_part_right = 0;

float d_part_left = 0;
float d_part_right = 0;

long pulseCount_l = 0;
long pulseCount_r = 0;

float linear_vel = 0.0; // linear velocity extracted from /cmd_vel topic
float angular_vel = 0.0; // angular velocity extracted from /cmd_vel topic

float desired_vel_meters_right = 0;
float desired_vel_meters_left = 0;

int sample_size_vel = 1000 / control_loop_time;
RunningAverage measuredVelRunningAverageLeft(sample_size_vel);
RunningAverage measuredVelRunningAverageRight(sample_size_vel);

float measured_vel_meters_right = 0;
float measured_vel_meters_left = 0;

float desired_vel_ticks_right = 0;
float desired_vel_ticks_left = 0;

int measured_vel_ticks_right = 0;
int measured_vel_ticks_left = 0;

int error_vel_ticks_right = 0;
int error_vel_ticks_left = 0;

bool desired_right_wheel_forwards = true;
bool desired_left_wheel_forwards = true;

bool currently_right_wheel_forwards = true;
bool currently_left_wheel_forwards = true;

bool stopping_robot = false;
