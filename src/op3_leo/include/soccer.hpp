/* Author: Pedro y Marlene */
#ifndef SOCCER_H
#define SOCCER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
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

void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
void goAction(int page);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();
bool isActionRunning();

void callbackJointStates(const sensor_msgs::JointState& posicion);
void callbackError(const geometry_msgs::Point& msg);
void callbackPosition(const geometry_msgs::Point& msg);
void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);

void startSoccer(float posiciones2[40][6], int ult_pos); 
void stop(float posiciones2[40][6], int ult_pos);
void kickRight(float posiciones2[40][6], int ult_pos);
void kickPenaltyRight(float posiciones2[40][6], int ult_pos);
void walkForward() ;
void turnLeft(float posiciones2[40][6], int ult_pos);
void turnRight(float posiciones2[40][6], int ult_pos);

const double FALL_FORWARD_LIMIT = 60;
const double FALL_BACK_LIMIT = -60;
double present_pitch_;
int page;
int state;

int ult_pos;
double rest_inc = 0.2618;
double rest_inc_giro = 0.1745;

double t_ref_ang;
double t_ref;
double act_val = 0;

double errorx;
double errory;

double positionx;
double positiony;
double area;

double head_pan;
double head_tilt;

float posiciones2[40][6];
const int rows = 40;
const int cols = 6;

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher action_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Subscriber imu_sub;
ros::Subscriber read_joint_sub;
ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;

int control_module = None;
bool demo_ready = false;

#endif 
