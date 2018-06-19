// my first program in C++
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/callback_queue.h>

class Robot {
    std_msgs::Float64 speed;
    std_msgs::Float64 speed_neg;
    std::vector<float> hokuyo_info;
    std::vector<float> hokuyo_info_no_treatment;
    std_msgs::Float32 hokuyo_inc_angle;
    std_msgs::Float64 stop_speed;


    ros::Publisher traction_1c_path, traction_1f_path, traction_2f_path, traction_1b_path, traction_2b_path;

    ros::Publisher joint_1f_path, joint_1b_path, joint_2f_path, joint_2b_path;

    ros::Publisher claw_1f_path, claw_2f_path, claw_1b_path, claw_2b_path, claw_1c_path;

    ros::Subscriber hokuyo_path;

    ros::NodeHandle nh;

    double dur_timer;
  public:
    Robot (void);
    void IHM_Panel(void);
    void move_forward(void);
    void move_backward(void);
    void change_speed(void);
    void change_time(void);
    void detect_obs(void);
    void aprox_obs(float dist);
    void surpass_obs(void);
    void callback_hokuyo(const sensor_msgs::LaserScan::ConstPtr& data);
};

Robot::Robot(void) {
  speed.data = 15;
  speed_neg.data = -speed.data;
  dur_timer = 3;
  hokuyo_inc_angle.data = 7;
  stop_speed.data = 0;

  traction_1c_path = nh.advertise<std_msgs::Float64>("/robot/traction_1c_controller/command", 1000);
  traction_1f_path = nh.advertise<std_msgs::Float64>("/robot/traction_1f_controller/command", 1000);
  traction_2f_path = nh.advertise<std_msgs::Float64>("/robot/traction_2f_controller/command", 1000);
  traction_1b_path = nh.advertise<std_msgs::Float64>("/robot/traction_1b_controller/command", 1000);
  traction_2b_path = nh.advertise<std_msgs::Float64>("/robot/traction_2b_controller/command", 1000);

  joint_1f_path = nh.advertise<std_msgs::Float64>("/robot/joint_1f_controller/command", 1000);
  joint_1b_path = nh.advertise<std_msgs::Float64>("/robot/joint_1b_controller/command", 1000);
  joint_2f_path = nh.advertise<std_msgs::Float64>("/robot/joint_2f_controller/command", 1000);
  joint_2b_path = nh.advertise<std_msgs::Float64>("/robot/joint_2b_controller/command", 1000);

  claw_1f_path = nh.advertise<std_msgs::Float64>("/robot/claw_1f_controller/command", 1000);
  claw_2f_path = nh.advertise<std_msgs::Float64>("/robot/claw_2f_controller/command", 1000);
  claw_1b_path = nh.advertise<std_msgs::Float64>("/robot/claw_1b_controller/command", 1000);
  claw_2b_path = nh.advertise<std_msgs::Float64>("/robot/claw_2b_controller/command", 1000);
  claw_1c_path = nh.advertise<std_msgs::Float64>("/robot/claw_1c_controller/command", 1000);

  hokuyo_path = nh.subscribe<sensor_msgs::LaserScan>("/robot/scan", 1000, &Robot::callback_hokuyo,this);

  Robot::IHM_Panel();
}

void Robot::IHM_Panel(void) {
  int choice;
  // ROS_INFO("Current Speed: %.2f", speed.data);
  // ROS_INFO("Duration of movement(sec): %.2f", dur_timer);
  // ROS_INFO("Current time: %.2f", ros::Time::now().toSec());
  // ROS_INFO("To move forward press 1");
  // ROS_INFO("To move backwards press 2");
  // ROS_INFO("To change the speed press 3");
  // ROS_INFO("To change the duration of the movement press 4");
  // ROS_INFO("To move until find, approx. and surpass an obstacle press 5");
  // ROS_INFO("To quit press 6");
  // ROS_INFO("Insert the number:");

  std::cout << "\nCurrent Speed: " << speed.data << "\n";
  std::cout << "Duration of movement in sec: " << dur_timer << "\n";
  std::cout << "current time: " << ros::Time::now().toSec() << "\n\n";
  std::cout << "To move forward press 1" << "\n";
  std::cout << "To move backwards press 2" << "\n";
  std::cout << "To change the speed press 3" << "\n";
  std::cout << "To change the duration of the movement press 4" << "\n";
  std::cout << "To move until find, approx. and surpass an obstacle press 5" << "\n";
  std::cout << "To quit press 6" << "\n";
  std::cout << "Insert the number here: ";
  std::cin >> choice;
  std::cout << "\n";
  if (choice == 1) {
    Robot::move_forward();
  }
  else if (choice == 2) {
    Robot::move_backward();
  }
  else if (choice == 3) {
    Robot::change_speed();
  }
  else if (choice == 4) {
    Robot::change_time();
  }
  else if (choice == 5) {
    Robot::detect_obs();
  }
  else if (choice == 6) {
    exit(1);
  }
  else {
    std::cout << "There isn't this option, moving back to choice panel" << "\n";
    Robot::IHM_Panel();
  }
}

void Robot::move_forward(void) {
  std::cout << "Moving Forward!" <<"\n";
  double current_time, starting_time, actual_time;

  traction_1c_path.publish(speed);
  traction_1f_path.publish(speed_neg);
  traction_2f_path.publish(speed);
  traction_1b_path.publish(speed_neg);
  traction_2b_path.publish(speed);
  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < dur_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }
  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);
  std::cout << "Moving back to panel" <<"\n";
  Robot::IHM_Panel();
}

void Robot::move_backward(void) {
  std::cout << "Moving Backwards!" <<"\n";
  double current_time, starting_time, actual_time;
  traction_1c_path.publish(speed_neg);
  traction_1f_path.publish(speed);
  traction_2f_path.publish(speed_neg);
  traction_1b_path.publish(speed);
  traction_2b_path.publish(speed_neg);
  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < dur_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }
  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);
  std::cout << "Moving back to panel" <<"\n";
  Robot::IHM_Panel();
}

void Robot::change_speed(void) {
  std::cout << "Insert the new speed value: ";
  std::cin >> speed.data;
  speed.data = abs(speed.data);
  speed_neg.data = -speed.data;
  std::cout << "Moving back to panel" <<"\n";
  Robot::IHM_Panel();
}

void Robot::change_time(void) {
  std::cout << "Insert new duration time of movement: ";
  std::cin >> dur_timer;
  dur_timer = abs(dur_timer);
  std::cout << "Moving back to panel" <<"\n";
  Robot::IHM_Panel();
}

void Robot::detect_obs(void) {
  float largura = 0;
  float soma = 0;
  float dist;
  while ((largura*1000) < 30){
    soma = 0;
    ROS_INFO("Obstacle not detected, proceding!");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
    for (int pos = 0; pos < hokuyo_info.size() ; pos++){
      soma = soma + hokuyo_info[pos];
    }
    largura = hokuyo_inc_angle.data*soma;
    ROS_INFO("largura lida em mm: %.2f", largura*1000);
    traction_1c_path.publish(speed);
    traction_1f_path.publish(speed_neg);
    traction_2f_path.publish(speed);
    traction_1b_path.publish(speed_neg);
    traction_2b_path.publish(speed);
  }
  ROS_INFO("Obstacle detected, preparing to aprox. it");
  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);
  dist = sqrt(pow(soma/hokuyo_info.size(),2)-pow(0.4,2));
  Robot::aprox_obs(dist);
  std::cout << "Finished all movements, Moving back to panel" <<"\n";
  Robot::IHM_Panel();
}

void Robot::aprox_obs(float dist) {
  std_msgs::Float64 speed_aprox;
  std_msgs::Float64 speed_neg_aprox;
  float timer_aprox;
  double current_time, starting_time, actual_time;

  speed_aprox.data = 5;
  speed_neg_aprox.data = -speed_aprox.data;
  timer_aprox = ((dist-0.08)/speed_aprox.data)*60;

  traction_1c_path.publish(speed_aprox);
  traction_1f_path.publish(speed_neg_aprox);
  traction_2f_path.publish(speed_aprox);
  traction_1b_path.publish(speed_neg_aprox);
  traction_2b_path.publish(speed_aprox);

  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < timer_aprox){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);
  ROS_INFO("Aprox. completed, preparing to surpass it");
  Robot::surpass_obs();
}

void Robot::surpass_obs(void) {
  float surpass_timer;
  double current_time, starting_time, actual_time;
  float dist;
  std_msgs::Float64 speed_surpass;
  std_msgs::Float64 speed_neg_surpass;
  std_msgs::Float64 auxiliar_var_1,auxiliar_var_2;

  //move up front arm to allow claws to rotate
  auxiliar_var_1.data = -0.3;
  auxiliar_var_2.data = 0.1;
  joint_1f_path.publish(auxiliar_var_1);
  joint_2f_path.publish(auxiliar_var_2);

  //Create a delay, to allow the arm to go up before rotating claws
  surpass_timer = 1;
  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  //Rotate claws
  auxiliar_var_1.data = 3.14;
  auxiliar_var_2.data = 3.14;
  claw_1f_path.publish(auxiliar_var_1);
  claw_2f_path.publish(auxiliar_var_2);

  //Move some distance to allow front arm to pass the obstacle
  dist = 0.6;
  speed_surpass.data = 5;
  speed_neg_surpass.data = -speed_surpass.data;
  surpass_timer = ((dist-0.08)/speed_surpass.data)*60;

  traction_1c_path.publish(speed_surpass);
  traction_1f_path.publish(speed_neg_surpass);
  traction_2f_path.publish(speed_surpass);
  traction_1b_path.publish(speed_neg_surpass);
  traction_2b_path.publish(speed_surpass);

  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);

  //rotate the claws back to original position
  auxiliar_var_1.data = 0;
  auxiliar_var_2.data = 0;
  claw_1f_path.publish(auxiliar_var_1);
  claw_2f_path.publish(auxiliar_var_2);

  //small delay to give sufficient time to claws rotate back to position
  surpass_timer = 0.5;
  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  //move front arm back to original position
  auxiliar_var_1.data = 0;
  auxiliar_var_2.data = 0;
  joint_1f_path.publish(auxiliar_var_1);
  joint_2f_path.publish(auxiliar_var_2);

  //First arm passed Obstacle

  //move both arms down to force the center up
  auxiliar_var_1.data = 1.3;
  auxiliar_var_2.data = 1.3;
  joint_1f_path.publish(auxiliar_var_1);
  joint_1b_path.publish(auxiliar_var_2);

  //Create a delay, to allow the center to go up before rotating claw
  surpass_timer = 1;
  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }
  //Rotate center claw
  auxiliar_var_1.data = 3.14;
  claw_1c_path.publish(auxiliar_var_1);

  //Move some distance to allow the center to pass the obstacle
  surpass_timer = ((dist-0.08)/speed_surpass.data)*60;

  traction_1c_path.publish(speed_surpass);
  traction_1f_path.publish(speed_neg_surpass);
  traction_2f_path.publish(speed_surpass);
  traction_1b_path.publish(speed_neg_surpass);
  traction_2b_path.publish(speed_surpass);

  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);

  //Rotate the center claw back to orginal position and arms back to positio to lower the cente
  auxiliar_var_1.data = 0;
  claw_1c_path.publish(auxiliar_var_1);
  joint_1f_path.publish(auxiliar_var_1);
  joint_1b_path.publish(auxiliar_var_1);

  //Central part passed Obstacle

  //move up back arm to allow claws to rotate
  auxiliar_var_1.data = -0.3;
  auxiliar_var_2.data = 0.1;
  joint_1b_path.publish(auxiliar_var_1);
  joint_2b_path.publish(auxiliar_var_2);

  //Create a delay, to allow the arm to go up before rotating claws
  surpass_timer = 1;
  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  //Rotate claws
  auxiliar_var_1.data = 3.14;
  auxiliar_var_2.data = 3.14;
  claw_1b_path.publish(auxiliar_var_1);
  claw_2b_path.publish(auxiliar_var_2);


  //Move some distance to allow front arm to pass the obstacle
  surpass_timer = ((dist-0.08)/speed_surpass.data)*60;

  traction_1c_path.publish(speed_surpass);
  traction_1f_path.publish(speed_neg_surpass);
  traction_2f_path.publish(speed_surpass);
  traction_1b_path.publish(speed_neg_surpass);
  traction_2b_path.publish(speed_surpass);

  current_time = 0;
  starting_time = ros::Time::now().toSec();
  while (current_time < surpass_timer){
    actual_time = ros::Time::now().toSec();
    current_time = actual_time - starting_time;
  }

  traction_1c_path.publish(stop_speed);
  traction_1f_path.publish(stop_speed);
  traction_2f_path.publish(stop_speed);
  traction_1b_path.publish(stop_speed);
  traction_2b_path.publish(stop_speed);

  //move claws and arm back to original position
  auxiliar_var_1.data = 0;
  claw_1b_path.publish(auxiliar_var_1);
  claw_2b_path.publish(auxiliar_var_1);
  joint_1b_path.publish(auxiliar_var_1);
  joint_2b_path.publish(auxiliar_var_1);

  ROS_INFO("Surpassed!");
}

void Robot::callback_hokuyo(const sensor_msgs::LaserScan::ConstPtr& data){

  hokuyo_info_no_treatment = data->ranges;
  hokuyo_info.clear();
  for(int i=0 ; i < hokuyo_info_no_treatment.size() ; i++){
    if (!(std::isinf(hokuyo_info_no_treatment[i]))){
      hokuyo_info.push_back(hokuyo_info_no_treatment[i]);
    }
  }
  // for(int i=0 ; i < hokuyo_info.size() ; i++){
  //   std::cout << hokuyo_info[i];
  // }
  // std::cout << "\n"<< "\n"<< "\n"<< "\n";
  // ROS_INFO("I heard: Enter");
  hokuyo_inc_angle.data = data->angle_increment;


}

void callback_hokuyo2(const sensor_msgs::LaserScan::ConstPtr& data){
  ROS_INFO("I heard: Enter");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Piro3_move");
  Robot piro3;


  // ros::NodeHandle n;
  // ros::Subscriber hokuyo_path2;
  // hokuyo_path2 = n.subscribe<sensor_msgs::LaserScan>("/robot/scan", 1000, callback_hokuyo2);
  // ros::Rate rate(10.0);
  // while(ros::ok()){
  //   ros::spinOnce(); // this is where the magic happens!!
  //   rate.sleep();
  // }

  return 0;
}
