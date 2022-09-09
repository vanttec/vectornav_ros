#include <iostream>
#include <math.h>
#include <vector>
#include "ros/ros.h"	
#include "ros/time.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"

using namespace std;

class sdvPathNodeClass
{
	public:
		geometry_msgs::Pose2D receivedPose;
		geometry_msgs::Pose poseToAppend;
		geometry_msgs::PoseStamped stampedPoseToAppend;
		nav_msgs::Path pathToPublish;
		uint32_t pathSeq;
		uint32_t poseSeq;
		

	

	void initialize_path_to_publish(){
		sdv_path_pub = n.advertise<nav_msgs::Path>("/vectornav/rviz_data/path", 1000);
		sdv_ned_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &sdvPathNodeClass::NED_pose_callback, this);


		pathToPublish.header.frame_id = "world";
		pathSeq = 0;
		poseSeq = 0;
		pathToPublish.header.stamp = ros::Time::now();
		pathToPublish.poses.reserve(100*sizeof(geometry_msgs::PoseStamped));
	}


	void NED_pose_callback(const geometry_msgs::Pose2D::ConstPtr& _msg){
		
		
		receivedPose.theta = _msg -> theta;
		receivedPose.x = _msg -> x;
		receivedPose.y = _msg -> y;
		
	}


	void update_path()
	{
		pathToPublish.header.seq = pathSeq++;
		pathToPublish.header.stamp = ros::Time::now();
		pathToPublish.poses.push_back(stampedPoseToAppend);
		sdv_path_pub.publish(pathToPublish);
	}

	void obtain_pose(){
		poseToAppend.orientation.w = 0;
		poseToAppend.orientation.x = 0;
		poseToAppend.orientation.y = 0;
		poseToAppend.orientation.z = 0;
		poseToAppend.position.z = 0;


		poseToAppend.position.x = receivedPose.x;
		poseToAppend.position.y = -receivedPose.y;
	}

	void stamp_pose(){
		stampedPoseToAppend.pose = poseToAppend;
		stampedPoseToAppend.header.frame_id = "world";
		stampedPoseToAppend.header.seq = poseSeq++;
		stampedPoseToAppend.header.stamp = ros::Time::now(); 
	}

	private:
		ros::NodeHandle n;
		ros::Publisher sdv_path_pub;
		ros::Subscriber sdv_ned_pose_sub;

};

int main(int argc, char *argv[])
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* part of the ROS system.
	*/
	ros::init(argc, argv, "imu_path_publisher");
	ros::Time::init();

	/*
	 *  The nodehandle fully initializes the node; when it is destroyed, it ends
	 */ 


	ros::Rate loop_rate(100);

	//Creation of initial path and frame

	sdvPathNodeClass sdvPathNode;
	sdvPathNode.initialize_path_to_publish();



  while (ros::ok())
  {
    ros::spinOnce();
	sdvPathNode.obtain_pose();
	sdvPathNode.stamp_pose();
	sdvPathNode.update_path();
    loop_rate.sleep();
  }


	return 0;
}