#include <iostream>
#include "vn/sensors.h"
#include "vn/thread.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace Eigen;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "imu");

  	ros::NodeHandle n;
	
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	const uint32_t SensorBaudrate = 115200;

	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	//ROS Publisher
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/vectornav/imu/imu", 1000);
	ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("/vectornav/imu/mag", 1000);
	ros::Publisher pre_pub = n.advertise<sensor_msgs::FluidPressure>("/vectornav/imu/pre", 1000);
	ros::Publisher tem_pub = n.advertise<sensor_msgs::Temperature>("/vectornav/imu/tem", 1000);
	ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("/vectornav/imu/gps", 1000);
	ros::Publisher ref_pub = n.advertise<sensor_msgs::NavSatFix>("/vectornav/imu/ref", 1000);
	ros::Publisher odo_pub = n.advertise<nav_msgs::Odometry>("/vectornav/imu/odo", 1000);

	InsStateLlaRegister ref; //reference (starting) data
	ref = vs.readInsStateLla();

	sensor_msgs::NavSatFix gps_ref;
	gps_ref.latitude = ref.position.x;
	gps_ref.longitude = ref.position.y;
	gps_ref.altitude = ref.position.z;

	InsStateEcefRegister Ecefref;
	Ecefref = vs.readInsStateEcef();

	Vector3f Pe_ref;
	Pe_ref << Ecefref.position.x,
			  Ecefref.position.y,
			  Ecefref.position.z;

	Matrix3f Rne;
	Rne << -sin(gps_ref.latitude) * cos(gps_ref.longitude), -sin(gps_ref.latitude) * sin(gps_ref.longitude), cos(gps_ref.latitude),
		   -sin(gps_ref.longitude), cos(gps_ref.longitude), 0,
		   -cos(gps_ref.latitude) * cos(gps_ref.longitude), -cos(gps_ref.latitude) * sin(gps_ref.longitude), -sin(gps_ref.latitude);

	ros::Rate loop_rate(100);

  while (ros::ok())
  {

	InsStateLlaRegister ins; //Inertial Navigation System (INS) variable declaration
	QuaternionMagneticAccelerationAndAngularRatesRegister quat;
	ImuMeasurementsRegister measure;

	InsStateEcefRegister Ecef; //INS with Ecef coordinates

	ins = vs.readInsStateLla(); //Variable that reads the INS data
	quat = vs.readQuaternionMagneticAccelerationAndAngularRates();
	measure = vs.readImuMeasurements();
	Ecef = vs.readInsStateEcef();

	sensor_msgs::Imu imu;
	sensor_msgs::MagneticField mag;
	sensor_msgs::NavSatFix gps;
	sensor_msgs::FluidPressure pre;
	sensor_msgs::Temperature tem;
	nav_msgs::Odometry odo;

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

	mag.magnetic_field.x = quat.mag.x;
	mag.magnetic_field.y = quat.mag.y;
	mag.magnetic_field.z = quat.mag.z;

	gps.latitude = ins.position.x;
	gps.longitude = ins.position.y;
	gps.altitude = ins.position.z;

	pre.fluid_pressure = measure.pressure;

	tem.temperature = measure.temp;


//local velocity and yaw rate
	Vector3f eta_dot; //vector declaration of eta' = [x' y' psi'] (3 DOF global reference frame)
	eta_dot << ins.velocity.x,
			ins.velocity.y,
			ins.velocity.z;

	Matrix3f J; //matrix of transformation between reference frames
	J << cos(ins.yawPitchRoll.z)*cos(ins.yawPitchRoll.y), -sin(ins.yawPitchRoll.z)*cos(ins.yawPitchRoll.x)+cos(ins.yawPitchRoll.z)*sin(ins.yawPitchRoll.y)*sin(ins.yawPitchRoll.x), sin(ins.yawPitchRoll.z)*sin(ins.yawPitchRoll.x)+cos(ins.yawPitchRoll.z)*cos(ins.yawPitchRoll.y)*sin(ins.yawPitchRoll.x),
		 sin(ins.yawPitchRoll.z)*cos(ins.yawPitchRoll.y), cos(ins.yawPitchRoll.z)*cos(ins.yawPitchRoll.x)+sin(ins.yawPitchRoll.z)*sin(ins.yawPitchRoll.y)*sin(ins.yawPitchRoll.x), -cos(ins.yawPitchRoll.z)*sin(ins.yawPitchRoll.x)+sin(ins.yawPitchRoll.z)*sin(ins.yawPitchRoll.y)*cos(ins.yawPitchRoll.x),
		 -sin(ins.yawPitchRoll.y), cos(ins.yawPitchRoll.y)*sin(ins.yawPitchRoll.x), cos(ins.yawPitchRoll.y)*cos(ins.yawPitchRoll.x);

	Vector3f upsilon; //vector upsilon = [u v r] (3 DOF local reference frame)
	upsilon = J.inverse()*eta_dot; //transformation into local reference frame
	float u = upsilon(0); //surge velocity
	float v = upsilon(1); //sway velocity
	float w = upsilon(2); //yaw rate
	odo.twist.twist.linear.x = u;
	odo.twist.twist.linear.y = v;
	odo.twist.twist.linear.z = w;

	odo.twist.twist.angular.x = ins.angularRate.x;
	odo.twist.twist.angular.y = ins.angularRate.y;
	odo.twist.twist.angular.z = ins.angularRate.z;

//Ecef coordinates
	Vector3f Pe;
	Pe <<   Ecef.position.x,
			Ecef.position.y,
			Ecef.position.z;

//NED from Ecef coordinates
	Vector3f NED;
	NED = Rne * (Pe - Pe_ref);
	float N = NED(0);
	float E = NED(1);
	float D = NED(2);

	odo.pose.pose.position.x = N;
	odo.pose.pose.position.y = E;
	odo.pose.pose.position.z = D;

	odo.pose.pose.orientation = imu.orientation;

//Data publishing
    imu_pub.publish(imu);
    mag_pub.publish(mag);
    pre_pub.publish(pre);
    tem_pub.publish(tem);
    gps_pub.publish(gps);
    ref_pub.publish(gps_ref);
    odo_pub.publish(odo);

    ros::spinOnce();

    loop_rate.sleep();
  }

	vs.disconnect();

	return 0;
}