#include <iostream>
#include <eigen3/Eigen/Dense>
#include "vn/sensors.hpp"
#include "vn/thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace Eigen;

int main(int argc, char *argv[])
{

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("vn_sdv");


	const string SensorPort = "/dev/ttyUSB0";      // Linux format for virtual (USB) serial port.
	const uint32_t SensorBaudrate = 115200;

	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	//ROS Publishers for each required sensor data
	
	auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/vectornav/imu/imu", 1000);
	auto gps_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/vectornav/imu/gps", 1000);
	auto ins_pos_pub = node->create_publisher<geometry_msgs::msg::Pose2D>("/vectornav/ins_2d/ins_pose", 1000);
	auto ins_vel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("ins_vel", 1000);
	auto ins_acc_pub = node->create_publisher<geometry_msgs::msg::Vector3>("ins_acc", 1000);
	auto ins_ar_pub = node->create_publisher<geometry_msgs::msg::Vector3>("ins_ar", 1000);
	auto local_vel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("/vectornav/ins_2d/local_vel", 1000);
	auto NED_pose_pub = node->create_publisher<geometry_msgs::msg::Pose2D>("/vectornav/ins_2d/NED_pose", 1000);
	auto ECEF_pose_pub = node->create_publisher<geometry_msgs::msg::Pose2D>("ECEF_pose", 1000);
	auto ins_ref_pub = node->create_publisher<geometry_msgs::msg::Pose2D>("/vectornav/ins_2d/ins_ref", 1000);
	auto ecef_ref_pub = node->create_publisher<geometry_msgs::msg::Vector3>("/vectornav/ins_2d/ecef_ref", 1000);

	//Transformation of coordinates Geodetic-Ecef-NED for the reference
	int R_Ea = 6378137; //Earth radious
	float eccentricity = 0.08181919; //The eccentricity of the geodetic plane
	InsStateLlaRegister ref; //reference (starting) data
	//TODO check InsState vs InsState
	ref = vs.readInsStateLla();

	InsStateEcefRegister Ecefref;
	Ecefref = vs.readInsStateEcef();
	
	float Ecefrefx = 0;
	float Ecefrefy = 0;
	float Ecefrefz = 0;
	float refposx = 0;
	float refposy = 0;

	for (int i=1;i<=20;i++)
	{
		Ecefrefx = Ecefrefx + Ecefref.position.x;
		Ecefrefy = Ecefrefy + Ecefref.position.y;
		Ecefrefz = Ecefrefz + Ecefref.position.z;
		refposx = refposx + ref.position.x;
		refposy = refposy + ref.position.y;
	}

	Ecefrefx = Ecefrefx/20;
	Ecefrefy = Ecefrefy/20;
	Ecefrefz = Ecefrefz/20;
	refposx = refposx/20;
	refposy = refposy/20;

	float refx = (3.141592 / 180)*(refposx);
	float refy = (3.141592 / 180)*(refposy);

	Vector3f Pe_ref;
	Pe_ref << Ecefrefx,
			  Ecefrefy,
			  Ecefrefz;

	Matrix3f Rne;
	Rne << -sin(refx) * cos(refy), -sin(refx) * sin(refy), cos(refx),
		   -sin(refy), cos(refy), 0,
		   -cos(refx) * cos(refy), -cos(refx) * sin(refy), -sin(refx);

	geometry_msgs::msg::Pose2D ins_ref;
	ins_ref.x = refposx;
	ins_ref.y = refposy;
	ins_ref.theta = (3.141592 / 180)*(ref.yawPitchRoll.x);

	geometry_msgs::msg::Vector3 ecef_ref;
	ecef_ref.x = Ecefrefx;
	ecef_ref.y = Ecefrefy;
	ecef_ref.z = Ecefrefz;

	rclcpp::Rate loop_rate(100);

  while (rclcpp::ok())
  {
	InsStateLlaRegister ins; //Inertial Navigation System (INS) variable declaration
	QuaternionMagneticAccelerationAndAngularRatesRegister quat;
	InsStateEcefRegister Ecef; //INS with Ecef coordinates

	ins = vs.readInsStateLla(); //Variable that reads the INS data
	quat = vs.readQuaternionMagneticAccelerationAndAngularRates();
	Ecef = vs.readInsStateEcef();


	sensor_msgs::msg::Imu imu;	//NEW TOPIC
	sensor_msgs::msg::NavSatFix gps;	// NEW TOPIC
	geometry_msgs::msg::Pose2D ins_pose; //inertial navigation system pose (latitude, longitude, yaw)
	geometry_msgs::msg::Vector3 ins_vel; //velocity/speed in a global reference frame
	geometry_msgs::msg::Vector3 ins_acc; //acceleration
	geometry_msgs::msg::Vector3 ins_ar; //angular rate
	geometry_msgs::msg::Vector3 local_vel; //veocity/speed in a local reference frame
	geometry_msgs::msg::Pose2D NED_pose; //pose in a local reference frame (x, y, yaw)
	geometry_msgs::msg::Pose2D ECEF_pose;


	imu.orientation.x = quat.quat.x;
	imu.orientation.y = quat.quat.y;
	imu.orientation.z = quat.quat.z;
	imu.orientation.w = quat.quat.w;

	imu.angular_velocity.x = ins.angularRate.x; //roll rate
	imu.angular_velocity.y = ins.angularRate.y; //pitch rate
	imu.angular_velocity.z = ins.angularRate.z; //yaw rate (r)

	imu.linear_acceleration.x = ins.accel.x; //acceleration in the x axis
	imu.linear_acceleration.y = ins.accel.y; //acceleration in the y axis
	imu.linear_acceleration.z = ins.accel.z; //acceleration in the z axis




	ins_pose.x = ins.position.x; //latitude
	ins_pose.y = ins.position.y; //longitude
	ins_pose.theta = (3.141592 / 180)*(ins.yawPitchRoll.x); //yaw converted from degrees into radians

	float rad_pose_x = (3.141592 / 180)*(ins.position.x); //latitude in radians
	float rad_pose_y = (3.141592 / 180)*(ins.position.y); //longitude in radians

	ins_vel.x = ins.velocity.x; //velocity in North direction
	ins_vel.y = ins.velocity.y; //velocity in East direction
	ins_vel.z = ins.velocity.z; //velocity in Down direction

	ins_acc.x = ins.accel.x; //acceleration in the x axis
	ins_acc.y = ins.accel.y; //acceleration in the y axis
	ins_acc.z = ins.accel.z; //acceleration in the z axis

	ins_ar.x = ins.angularRate.x; //roll rate
	ins_ar.y = ins.angularRate.y; //pitch rate
	ins_ar.z = ins.angularRate.z; //yaw rate (r)

	gps.latitude = ins.position.x;
	gps.longitude = ins.position.y;
	gps.altitude = ins.position.z;

//local velocity and yaw rate
	Vector3f eta_dot; //vector declaration of eta' = [x' y' psi'] (3 DOF global reference frame)
	eta_dot << ins_vel.x,
			  ins_vel.y,
			  ins_ar.z;
	Matrix3f J; //matrix of transformation between reference frames
	J << cos(ins_pose.theta), -sin(ins_pose.theta), 0,
		 sin(ins_pose.theta), cos(ins_pose.theta), 0,
		 0, 0, 1;
	Vector3f upsilon; //vector upsilon = [u v r] (3 DOF local reference frame)
	upsilon = J.inverse()*eta_dot; //transformation into local reference frame
	float u = upsilon(0); //surge velocity
	float v = upsilon(1); //sway velocity
	float r = upsilon(2); //yaw rate
	local_vel.x = u;
	local_vel.y = v;
	local_vel.z = r;

//Ecef coordinates
	Vector3f Pe;
	Pe <<   Ecef.position.x,
			Ecef.position.y,
			Ecef.position.z;

	ECEF_pose.x = Pe(0);
	ECEF_pose.y = Pe(1);
	ECEF_pose.theta = ins_pose.theta;

//NED from Ecef coordinates
	Eigen::  Vector3f NED;
	NED = Rne * (Pe - Pe_ref);
	float N = NED(0);
	float E = NED(1);
	NED_pose.x = N;
	NED_pose.y = E;
	NED_pose.theta = ins_pose.theta;

//Data publishing
	imu_pub -> publish(imu);	//IMU topic
	gps_pub -> publish(gps);	//IMU topic
    ins_pos_pub -> publish(ins_pose);
    ins_vel_pub -> publish(ins_vel);
    ins_acc_pub ->publish(ins_acc);
    ins_ar_pub -> publish(ins_ar);
    local_vel_pub -> publish(local_vel);
    NED_pose_pub -> publish(NED_pose);
    //ECEF_pose_pub.publish(ECEF_pose);
    ins_ref_pub -> publish(ins_ref);
    //ecef_ref_pub.publish(ecef_ref);

    rclcpp::spin_some(node);

    loop_rate.sleep();
  }

	vs.disconnect();

	return 0;
}