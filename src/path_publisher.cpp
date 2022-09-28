/*
@file :        path_publisher.cpp
@date:         Fri Sep 23, 2022
@date_modif:   Fri Sep 23, 2022
@author:       David Rogelio Saláis Carrasco
@e-mail:	   rogelio.salais@hotmail.com
@brief:		   This script publishes a path topic for visualization in RVIZ 
				directly from NED_pose information obtained from the VN-3OO sensor
				For use in the SDV project
@version:
Copyright		Open Source 
*/

#include <iostream>
#include <math.h>
#include <vector>
#include "ros/ros.h"	
#include "ros/time.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"


class SdvPathNodeClass
{
	public:

	
	void initializePath(){
		sdv_path_pub = n.advertise<nav_msgs::Path>("/vectornav/rviz_data/path", 1000);
		sdv_ned_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &SdvPathNodeClass::nedPoseCallback, this);
		path_to_publish.header.frame_id = "world";
		path_to_publish.header.stamp = ros::Time::now();
		path_to_publish.poses.reserve(100*sizeof(geometry_msgs::PoseStamped));
	}


	void nedPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
		
		received_pose.theta = msg -> theta;
		received_pose.x = msg -> x;
		received_pose.y = msg -> y;
		auxiliary_quaternion -> setRPY(M_PI, 0, -msg->theta);
		
	}


	void updatePath()
	{
		path_to_publish.header.seq = path_counter++;
		path_to_publish.header.stamp = ros::Time::now();
		path_to_publish.poses.push_back(stamped_pose_to_append);
		sdv_path_pub.publish(path_to_publish);
	}

	
	void obtainAndStampPose(){
		stamped_pose_to_append.pose.orientation.w = 1;
		stamped_pose_to_append.pose.orientation.x = auxiliary_quaternion -> getX();
		stamped_pose_to_append.pose.orientation.y = auxiliary_quaternion -> getY();
		stamped_pose_to_append.pose.orientation.z = auxiliary_quaternion -> getZ();
		stamped_pose_to_append.pose.position.z = 0;
		stamped_pose_to_append.pose.position.x = received_pose.x;
		stamped_pose_to_append.pose.position.y = -received_pose.y;
		stamped_pose_to_append.header.frame_id = "world";
		stamped_pose_to_append.header.seq = pose_counter++;
		stamped_pose_to_append.header.stamp = ros::Time::now(); 
	}


	private:
		ros::NodeHandle n;
		ros::Publisher sdv_path_pub;
		ros::Subscriber sdv_ned_pose_sub;
		geometry_msgs::Pose2D received_pose;
		geometry_msgs::PoseStamped stamped_pose_to_append;
		nav_msgs::Path path_to_publish;
		tf2::Quaternion *auxiliary_quaternion;
		uint32_t path_counter = 0;
		uint32_t pose_counter = 0;

};

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "imu_path_publisher");
	ros::Time::init();
	ros::Rate loop_rate(100);

	//Creation of initial path and frame

	SdvPathNodeClass sdvPathNode;
	sdvPathNode.initializePath();



  while (ros::ok())
  {
    ros::spinOnce();
	sdvPathNode.obtainAndStampPose();
	sdvPathNode.updatePath();
    loop_rate.sleep();
  }
	return 0;
}