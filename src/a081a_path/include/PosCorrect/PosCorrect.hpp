#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // include the LaserScan msgs definition //#
#include <nav_msgs/Odometry.h> // include the odometry msgs definition
#include <geometry_msgs/Twist.h> // include the robot control msgs definition
#include <std_msgs/Float32.h> // include the msgs needed to be published
#include <std_msgs/Bool.h>
#include <gazebo_msgs/LinkStates.h> //# for true posiiton
#include <geometry_msgs/Pose.h> 
#include <ros/console.h>
#include <math.h> //#

static const double PI = 3.1415;

namespace poscorrect{
class PosCorrect{

private:

	ros::NodeHandle nodehandle_; 

	// topics to be subscribed
    // ros::Subscriber scan_sub_; // laser scan //#
    // ros::Subscriber odom_sub_; // odometry
    ros::Subscriber true_sub_; // odometry
    ros::Subscriber start_correction_sub_;
	// topics to be published
	ros::Publisher vel_pub_; // twist control
	ros::Publisher error_forward_pub_; // log
	ros::Publisher error_angle_pub_; //log
	ros::Publisher control_signal_forward_pub_; // log
	ros::Publisher control_signal_angle_pub_;
	ros::Publisher dist_from_target_pub_;

	double pos_x_, pos_y_; //# moved q_z_ to callback
	double ang_z_; // eular angle from quaternion q_z
	


	geometry_msgs::Twist vel_cmd_; // control the robot msgs
	double trans_forward_, trans_angle_; // pid output

	// PID related
	double error_forward_, error_angle_, error_forward_prev_, error_angle_prev_;
	double I_forward_, I_angle_; // integral part
	double D_forward_, D_angle_; // derivative part

	bool kill=false;

	// void odomCallBack(const nav_msgs::OdometryConstPtr& odomMsg);
    //	void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
	void trueCallBack(const nav_msgs::OdometryConstPtr& odomMsg);
	void corrCallBack(const std_msgs::Bool& checkerMsg);
	void pidAlgorithm();
	bool loadParam();


public:

	double dt; //variable
	double target_distance, target_angle;
	double Kp_f, Ki_f, Kd_f;
	double Kp_angle, Ki_angle, Kd_angle;
    double pillar_x, pillar_y; //# pillar position
	bool corr_checker;

	PosCorrect(ros::NodeHandle& nh);
	virtual ~PosCorrect();

	void spin();

};

}
