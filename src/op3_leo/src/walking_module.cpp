/* Author: Marlene Cobian */

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/GetWalkingParam.h"

void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
void goAction(int page);
void goWalk(std::string& command);
bool isActionRunning();
bool getWalkingParam();

bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);

double rest_inc = 0.2181;
//rest_inc =0.2618 15°
//double rest_inc_giro = 0.08726;

double alpha = 0.4;
double pitch;

double rpy_orientation;
const double FALL_FORWARD_LIMIT = 55;
const double FALL_BACK_LIMIT = -55;
double present_pitch_;
int page;
int state;

const int row = 5700;
int col = 14;
float posiciones[row][col];

const int row2 = 40;
int col2 = 6;
float posiciones2[row2][col2];

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Publisher vision_case_pub;
ros::Publisher action_pose_pub;
ros::Publisher walk_command_pub;
ros::Subscriber read_joint_sub;
ros::Subscriber imu_sub;

ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;
ros::ServiceClient get_param_client;

int control_module = None;
bool demo_ready = false;

bool find_ball = true;

//node main
int main(int argc, char **argv)
{
  
  //init ros
  ros::init(argc, argv, "read_write");
  ros::NodeHandle nh(ros::this_node::getName());

  //subscribers
  //ros::Subscriber lectura = nh.subscribe("/robotis/present_joint_states",1,CallBack);
  //ros::Subscriber error_sub = nh.subscribe("/error", 5, callbackError);
  //ros::Subscriber position_sub = nh.subscribe("/position", 5, callbackPosition);
  //ros::Subscriber joint_error_sub = nh.subscribe("/robotis/present_joint_states", 5, CallBack);
  //ros::Subscriber find_ball_sub = nh.subscribe("/find_ball", 5, findballCallBack);
  imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, callbackImu);
  
  std::string command;
  
  std::ifstream myfile ("/home/robotis/nayarit_ws/src/op3_leo/data/Pararse.txt");
  if (myfile.is_open()){
	std::cout << "El archivo se abrió";
	
		for (int idx2 = 0; idx2 < row2; idx2++){
			for (int idy2 = 0; idy2 < col2; idy2++){
				myfile >> posiciones2[idx2][idy2];
			}
			
		}
		myfile.close();
  } else {
  	std::cout << "El archivo no abrió";
  }
  
  //publishers
  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
  vision_case_pub = nh.advertise<std_msgs::Bool>("/vision_case", 1000);
  action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  walk_command_pub = nh.advertise<std_msgs::Int32>("/robotis/walking/command", 0);
  
  //services
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
  is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  get_param_client = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //wait for starting of op3_manager
  std::string manager_name = "/op3_manager";
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true)
    {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  readyToDemo();

  //node loop
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  //pararse en posición para caminar
  ros::Duration(1).sleep();
  ros::Rate loop_rate_pararse(60);
  
  for (int fila2=0; fila2<row2; fila2++){
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(posiciones2[fila2][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(posiciones2[fila2][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(posiciones2[fila2][2] + rest_inc);
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(posiciones2[fila2][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(posiciones2[fila2][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(posiciones2[fila2][5] - rest_inc);
    write_joint_pub.publish(write_msg);
      
    loop_rate_pararse.sleep();
  }
  
 //////////////////////////////////////////////// acomodo de pies ////////////////////////////////////////////////
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
    
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);

 //////////////////////////////////////////////// loop ////////////////////////////////////////////////

  while (ros::ok()){
    ros::spinOnce();
    ros::Rate loop_rate(SPIN_RATE);

    std::string command = "start";
    goWalk(command);
    ros::Duration(3.0).sleep();
    std::string command = "stop";
    goWalk(command);
  }
  return 0;
}

void readyToDemo()
{
  ROS_INFO("Start read-write demo");
  torqueOnAll();
  ROS_INFO("Torque on all joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Go init pose");

  //wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

void goAction(int page) {
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

void goWalk(std::string& command) {
  setModule("walking_module");
  ROS_INFO("Walking");

  std_msgs::String command_msg;
  command_msg.data = command;
  walk_command_pub.publish(command_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }
  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll()
{
  std_msgs::String check_msg;
  check_msg.data = "check";
  dxl_torque_pub.publish(check_msg);
}

bool isActionRunning() {
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client.call(is_running_srv) == false) {
    ROS_ERROR("Failed to start action module");
    return true;
  } else {
    if (is_running_srv.response.is_running == true) {
      return true;
    }
  }
  return false;
}

bool getWalkingParam() {
  op3_walking_module_msgs::GetWalkingParam get_param_srv;

  if (get_param_client.call(get_param_srv) == false) {
    ROS_ERROR("Failed to get walking params");
    return true;
  } else {
    if (get_param_srv.request.get_param == true) {
      return true;
    }
  }
  return false;
}

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg) {
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / 3.141516);

  double pitch = rpy_orientation.coeff(1, 0);

  double alpha = 0.4;
  if (present_pitch_ == 0) 
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT) {
    goAction(122);
    setModule("none");
  } else if (present_pitch_ < FALL_BACK_LIMIT) {
    goAction(123);
    setModule("none");
  } else {
    state = 0;
  }
}