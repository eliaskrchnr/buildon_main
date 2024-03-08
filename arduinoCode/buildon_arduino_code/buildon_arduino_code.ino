#include "header.h"
#include "header_ros_nodes.h"


void setup() {
  // LED Pins
  pinMode(52, OUTPUT);
  digitalWrite(52, HIGH);
  
  pinMode(50, OUTPUT);
  digitalWrite(50, LOW);

  // setup ROS nodes
  nh.initNode();

  nh.advertise(arduinoHeartbeat);
  nh.advertise(debug1);
  nh.advertise(debug2);
  nh.advertise(left_wheel_vel);
  nh.advertise(right_wheel_vel);
  nh.advertise(desired_left_wheel_vel);
  nh.advertise(desired_right_wheel_vel);
  nh.advertise(pub_pwm_right);
  nh.advertise(pub_pwm_left);
  nh.advertise(pub_ticks_left);
  nh.advertise(pub_ticks_right);

  nh.advertise(pub_control_part_p);
  nh.advertise(pub_control_part_i);
  nh.advertise(pub_control_part_d);


  
  nh.subscribe(sub);
  nh.subscribe(cmd_vel_linear);
  nh.subscribe(cmd_vel_angular);
  nh.subscribe(Kp_value);
  nh.subscribe(Ki_value);
  nh.subscribe(Kd_value);
  nh.subscribe(set_control_loop_time);

  nh.subscribe(setTn);
  nh.subscribe(setTv);


  // setup left wheel pins
  pinMode(Enable_l, OUTPUT);
  pinMode(Direction_l, OUTPUT);
  pinMode(Encoder_l, INPUT);
  pinMode(Speed_l, OUTPUT);
  digitalWrite(Enable_l, HIGH); //Start
  digitalWrite(Direction_l, HIGH); //HIGH for forwards. !HIGH for backwards
  analogWrite(Speed_l, 0); //Speed

  // setup right wheel pins
  pinMode(Enable_r, OUTPUT);
  pinMode(Direction_r, OUTPUT);
  pinMode(Encoder_r, INPUT);
  pinMode(Speed_r, OUTPUT);
  digitalWrite(Enable_r, HIGH); //Start
  digitalWrite(Direction_r, !HIGH); //!HIGH for forwards. HIGH for backwards
  analogWrite(Speed_r, 0); //Speed

  // setup encoder. 45 ticks per rotation.
  attachInterrupt(digitalPinToInterrupt(Encoder_l), countPulsesL, RISING); // Attach interrupt on rising edge
  attachInterrupt(digitalPinToInterrupt(Encoder_r), countPulsesR, RISING); // Attach interrupt on rising edge
  Serial.begin(9600); // needs to be at end of setup loop
}

void loop() {

  if (millis() - loop_time_1 >= 1000) { //for debugging.
    loop_time_1 = millis();
    digitalWrite(52, HIGH - digitalRead(52));
    n += 1;
    arduino_hearbeat_msg.data = n;
    arduinoHeartbeat.publish( &arduino_hearbeat_msg );
  }


  if (millis() - last_loop_time > control_loop_time) {
    last_loop_time = millis();
    get_desired_tick_velocity(); // returns desired tick velocity of both wheels
    get_measured_tick_velocity(); // returns measured tick velocity of both wheels
    calc_pwm_output(); // calculates PWM output for both wheels based on PID control
  }

  if (millis() - last_ros_com_loop_time > ros_com_loop_time) {
    last_ros_com_loop_time = millis();

    publish_measured_velocity();
    
    pub_ticks_left_msg.data = pulseCount_l;
    pub_ticks_left.publish(&pub_ticks_left_msg);
    
    pub_ticks_right_msg.data = pulseCount_r;
    pub_ticks_right.publish(&pub_ticks_right_msg);} 

  if (millis() - last_cmd_vel_time >= 1000) {
    analogWrite(Speed_l, 0); //Speed
    analogWrite(Speed_r, 0); //Speed
    //Serial.println(millis() - last_cmd_vel_time);
    }
  else if (currently_right_wheel_forwards == desired_right_wheel_forwards && currently_left_wheel_forwards == desired_left_wheel_forwards) {
    send_pwm_signal();
    }
    
  else if (measured_vel_ticks_right == 0 && measured_vel_ticks_left == 0) {
    currently_right_wheel_forwards = desired_right_wheel_forwards;
    currently_left_wheel_forwards = desired_left_wheel_forwards;
    
    digitalWrite(Direction_l, desired_left_wheel_forwards);
    digitalWrite(Direction_r, !desired_right_wheel_forwards);
    
    send_pwm_signal();
    }
  else {
    analogWrite(Speed_l, 0); //Speed
    analogWrite(Speed_r, 0); //Speed
    }
    
  nh.spinOnce();
}


void countPulsesL() {
  if (currently_left_wheel_forwards == true) pulseCount_l = pulseCount_l + 1;
  else if (currently_left_wheel_forwards == false) pulseCount_l = pulseCount_l - 1;
}

void countPulsesR() {
  if (currently_right_wheel_forwards == false) pulseCount_r = pulseCount_r - 1;
  else pulseCount_r = pulseCount_r + 1;
}
