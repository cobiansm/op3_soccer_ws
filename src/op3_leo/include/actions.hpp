
#include "soccer.hpp"

void stop(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Detenerse
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows - 1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][2] + rest_inc);
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows - 1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][5] - rest_inc);
  write_joint_pub.publish(write_msg);
  ros::Duration(0.1).sleep();
}

void kickRight(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  stop(posiciones2, rows);

  //Inclinarse para alinear el centro de masa
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.17);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(-0.17);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(0.15);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.45);
  write_joint_pub.publish(write_msg);
  
  //Posicion de seguridad
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7091);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5287);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.9474 + rest_inc);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0);
  write_joint_pub.publish(write_msg);
  
  //Patada
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.0046);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-0.7420);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(1.2287 + rest_inc);
  write_joint_pub.publish(write_msg);
  
  //Posicion de seguridad
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7091);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5287);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.9474 + rest_inc);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0);
  write_joint_pub.publish(write_msg);
  
  //Regreso
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
  ros::Duration(0.01).sleep();

  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows - 1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][2] + rest_inc);
  write_joint_pub.publish(write_msg);
}

void kickPenaltyRight(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Detenerse
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows - 1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][2] + rest_inc);
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows - 1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][5] - rest_inc);
  write_joint_pub.publish(write_msg);
  
  //Inclinarse para alinear el centro de masa
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.17);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(-0.17);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(0.15);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.45);
  write_joint_pub.publish(write_msg);
  
  //Posicion de seguridad
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7091);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5287);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.9474 + rest_inc);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0);
  write_joint_pub.publish(write_msg);
  
  //Patada
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.0046);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-0.7420);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(1.2287 + rest_inc);
  write_joint_pub.publish(write_msg);

  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(0);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(1.5 + rest_inc);
  write_joint_pub.publish(write_msg);
  
  //Posicion de seguridad
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7091);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.8);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(1.4 + rest_inc);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0);
  write_joint_pub.publish(write_msg);
  
  //Regreso
  ros::Duration(0.3).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(0);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(0);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0);
  
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows - 1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][2] + rest_inc);
  write_joint_pub.publish(write_msg);
}

void walkForward() {
    sensor_msgs::JointState write_msg;
    write_msg.header.stamp = ros::Time::now();

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
    
    write_joint_pub.publish(write_msg);
}

void turnLeft(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Levantar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.7520);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.5317);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.8143 - rest_inc_giro);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0.1746*1.5);
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(-0.1746*1.5);

  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
  

  //Bajar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows - 1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][5]);
  write_joint_pub.publish(write_msg);
  
  //Levantar pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7520);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5317);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.8143 + rest_inc_giro);
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(0);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0);
  write_joint_pub.publish(write_msg);
  
  //Bajar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows - 1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][2] + rest_inc_giro);
  write_joint_pub.publish(write_msg);
}

void turnRight(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Levantar pie derecho 
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7091);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.4131);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.7091 + rest_inc_giro);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0.1746*1.5);
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(-0.1746*1.5);

  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);

  //Bajar pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows - 1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][2] + rest_inc_giro);
  write_joint_pub.publish(write_msg);
  
  //Levantar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.7091);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.4131);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.7091 - rest_inc_giro);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0);
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(0);
  write_joint_pub.publish(write_msg);
  
  //Bajar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows - 1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows - 1][5] - rest_inc_giro);
  write_joint_pub.publish(write_msg);
}
