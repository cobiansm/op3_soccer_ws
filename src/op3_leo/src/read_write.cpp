/* Author: Pedro y Marlene */

#include "soccer.hpp"
#include "actions.hpp"

//node main
int main(int argc, char **argv) {
  
  //init ros
  ros::init(argc, argv, "read_write");
  ros::NodeHandle nh(ros::this_node::getName());

  std::ifstream myfile ("/home/robotis/nobios/src/op3_leo/data/Pararse.txt");
  if (myfile.is_open()) {
	std::cout << "El archivo se abrio" << std::endl;
	
		for (int i = 0; i < rows; i++){
			for (int j = 0; j < cols; j++){
				myfile >> posiciones2[i][j];
			}
		}
		myfile.close();
  } else {
  	std::cout << "El archivo no abrio" << std::endl;
  }

  //subscribers
  ros::Subscriber imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, callbackImu);
  ros::Subscriber error_sub = nh.subscribe("/error", 5, callbackError);
  ros::Subscriber position_sub = nh.subscribe("/position", 5, callbackPosition);
  ros::Subscriber joint_error_sub = nh.subscribe("/robotis/present_joint_states", 5, callbackJointStates);
  
  //publishers
  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
    
  //services
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
  is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  
  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //wait for starting of op3_manager
  std::string manager_name = "/op3_manager";
  while (ros::ok()) {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true) {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  readyToDemo();

  //node loop
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  //standup
  ros::Duration(1).sleep();
  ros::Rate loop_rate_pararse(60);
  
  for (int i=0; i<rows; i++){
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(posiciones2[i][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(posiciones2[i][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(posiciones2[i][2] + rest_inc);
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(posiciones2[i][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(posiciones2[i][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(posiciones2[i][5] - rest_inc);
    write_joint_pub.publish(write_msg);
        
    loop_rate_pararse.sleep();
  }
    
  //feet arrange
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

    //ros ok
    while (ros::ok()) {
        ros::spinOnce();
        ros::Rate loop_rate(SPIN_RATE);
        write_msg.name.push_back("head_pan");
        write_msg.position.push_back(positionx);
        write_msg.name.push_back("head_tilt");
        write_msg.position.push_back(positiony);
        write_joint_pub.publish(write_msg);

        /*if (head_tilt == -0.5) {
          if (head_pan < 0) {
            goAction(61);
            setModule("none");
          } else {
            goAction(62);
            setModule("none");
          }
        }

      /*if (area > 65000 && /*head_pan <= -0.18 && head_pan >= -0.52 && head_tilt < -1.05) {
          std::cout  << "PATEAA SIUUUUUUU" << std::endl;
          kickRight(posiciones2, rows);
          ros::Duration(1).sleep();
      } else if(head_pan > -0.4 && head_pan < 0.4 && (errorx > -40 && errorx < 40) && (errory > -40 && errory < 40)){
          std::cout  << "Caminando ando :p" << std::endl;
          //Pie izquierdo
          ros::Duration(0.1).sleep();
          write_msg.name.push_back("l_ank_pitch");
          write_msg.position.push_back(0.7520);
          write_msg.name.push_back("l_knee");
          write_msg.position.push_back(1.5317);
          write_msg.name.push_back("l_hip_pitch");
          write_msg.position.push_back(-0.8143-rest_inc);
          write_joint_pub.publish(write_msg);
          
          ros::Duration(0.1).sleep();
          write_msg.name.push_back("l_ank_pitch");
          write_msg.position.push_back(0.5486);
          write_msg.name.push_back("l_knee");
          write_msg.position.push_back(1.1446);
          write_msg.name.push_back("l_hip_pitch");
          write_msg.position.push_back(-0.6618-rest_inc);
          write_msg.name.push_back("r_ank_pitch");	//Pie derecho se acomoda para que centro de masa quede en medio de ambos pies
          write_msg.position.push_back(-0.5845);
          write_msg.name.push_back("r_knee");
          write_msg.position.push_back(-1.1453);
          write_msg.name.push_back("r_hip_pitch");
          write_msg.position.push_back(0.5233+rest_inc);
          write_joint_pub.publish(write_msg);
          
          //Pie derecho
          ros::Duration(0.1).sleep();
          write_msg.name.push_back("r_ank_pitch");
          write_msg.position.push_back(-0.7520);
          write_msg.name.push_back("r_knee");
          write_msg.position.push_back(-1.5317);
          write_msg.name.push_back("r_hip_pitch");
          write_msg.position.push_back(0.8143+rest_inc);
          write_joint_pub.publish(write_msg);
          
          ros::Duration(0.1).sleep();
          write_msg.name.push_back("r_ank_pitch");	
          write_msg.position.push_back(-0.5486);		//-0.5486  <- valor matlab
          write_msg.name.push_back("r_knee");
          write_msg.position.push_back(-1.1446);
          write_msg.name.push_back("r_hip_pitch");
          write_msg.position.push_back(0.6618+rest_inc);
          write_msg.name.push_back("l_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
          write_msg.position.push_back(0.5845);
          write_msg.name.push_back("l_knee");
          write_msg.position.push_back(1.1453);
          write_msg.name.push_back("l_hip_pitch");
          write_msg.position.push_back(-0.5233-rest_inc);
      } else if (errorx > -10 && errorx < 10 && errory > -20 && errory < 20 && (head_pan < -0.1745 || head_pan > 0.1745)) {
          if (head_pan > 0) {
              std::cout  << "Izquierda :0" << std::endl;
              turnLeft(posiciones2, rows);
          } else {
              std::cout  << "Derechaaaa D:" << std::endl;
              turnRight(posiciones2, rows);
          }
      } else {
          std::cout  << "Quieto -_-" << std::endl;
          stop(posiciones2, rows);
      }*/
    }
  return 0;
}


void readyToDemo() {
  ROS_INFO("Start read-write demo");
  torqueOnAll();
  ROS_INFO("Torque on all joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Init pose");

  //wait while ROBOTIS-OP3 goes to the init posture
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose() {
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

/* pages OP3:
1: standup
9: walk ready
15: sit down
60: keeper ready
61: keeper right
62: keeper left
122: get up forward
123: get up backward
124: get down forward
125: get down backward
126: pushup
*/

void goAction(int page) {
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

bool checkManagerRunning(std::string& manager_name) {
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

void setModule(const std::string& module_name) {
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll() {
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

void callbackJointStates(const sensor_msgs::JointState& posicion){
  head_pan = posicion.position[0];
  head_tilt = posicion.position[1];
}

void callbackError(const geometry_msgs::Point& msg) {
  errorx = msg.x;
  errory = msg.y;
}

void callbackPosition(const geometry_msgs::Point& msg) {
  positionx = msg.x;
  positiony = msg.y;
  area = msg.z;
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
