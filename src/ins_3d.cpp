#include <iostream>
#include "vn/sensors.h"
#include "vn/thread.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace Eigen;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "ins_3d");

  	ros::NodeHandle n;
	
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	const uint32_t SensorBaudrate = 115200;

	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	//ROS Publishers for each required sensor data
	ros::Publisher ins_ypr_pub = n.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_ypr", 1000);
	ros::Publisher ins_acc_pub = n.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_acc", 1000);
	ros::Publisher ins_ar_pub = n.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_ar", 1000);
	//ros::Publisher local_vel_pub = n.advertise<geometry_msgs::Vector3>("local_vel", 1000);

	ros::Rate loop_rate(100);

  while (ros::ok())
  {

	InsStateLlaRegister ins; //Inertial Navigation System (INS) variable declaration

	ins = vs.readInsStateLla(); //Variable that reads the INS data

	geometry_msgs::Vector3 ins_ypr; //inertial navigation system pose (latitude, longitude, yaw)
	geometry_msgs::Vector3 ins_acc; //acceleration
	geometry_msgs::Vector3 ins_ar; //angular rate
	//geometry_msgs::Vector3 local_vel; //veocity/speed in a local reference frame

	ins_ypr.x = (3.141592 / 180)*(ins.yawPitchRoll.z); //roll converted from degrees into radians
	ins_ypr.y = (3.141592 / 180)*(ins.yawPitchRoll.y); //pitch converted from degrees into radians
	ins_ypr.z = (3.141592 / 180)*(ins.yawPitchRoll.x); //yaw converted from degrees into radians

	ins_acc.x = ins.accel.x; //acceleration in the x axis
	ins_acc.y = ins.accel.y; //acceleration in the y axis
	ins_acc.z = ins.accel.z; //acceleration in the z axis

	ins_ar.x = ins.angularRate.x; //roll rate
	ins_ar.y = ins.angularRate.y; //pitch rate
	ins_ar.z = ins.angularRate.z; //yaw rate

//ToDo: Create integral of accelerations to approximate speeds, if required

//Data publishing
    ins_ypr_pub.publish(ins_ypr);
    ins_acc_pub.publish(ins_acc);
    ins_ar_pub.publish(ins_ar);
    //local_vel_pub.publish(local_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }

	vs.disconnect();

	return 0;
}