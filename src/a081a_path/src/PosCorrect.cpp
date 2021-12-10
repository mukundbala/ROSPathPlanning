#include <PosCorrect/PosCorrect.hpp>
#include <ros/ros.h>

using namespace poscorrect;

PosCorrect::PosCorrect(ros::NodeHandle& nh) : nodehandle_(nh){

	//load the param
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

	// declare all the subscriber and publisher
	// odom_sub_ = nodehandle_.subscribe("/husky_velocity_controller/odom", 1, &BotControl::odomCallBack, this);
	true_sub_ = nodehandle_.subscribe("/odom", 1, &PosCorrect::trueCallBack, this);
	start_correction_sub_ = nodehandle_.subscribe("/start_corr",1,&PosCorrect::corrCallBack,this);


	vel_pub_ = nodehandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 200); 
	error_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_forward", 1); 
	error_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_angle", 1);
	control_signal_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_forward", 1);
	control_signal_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_angle", 1);
	dist_from_target_pub_ = nodehandle_.advertise<std_msgs::Float32>("/dist_from_target", 1);



	//initialize variables
	error_forward_ = 0;
	error_angle_ = 0;
	error_forward_prev_ = 0;
	error_angle_prev_ = 0;
	I_forward_ = 0;
	I_angle_ = 0;
	D_forward_ = 0;
	D_angle_ = 0;

	ROS_INFO("Node Initialized");
}

PosCorrect::~PosCorrect(){}

void PosCorrect::trueCallBack(const nav_msgs::OdometryConstPtr& odomMsg){ //an array containing all the messages
    pos_y_ = odomMsg->pose.pose.position.y;
  	pos_x_ = odomMsg->pose.pose.position.x;
	float q_x = odomMsg->pose.pose.orientation.x;
  	float q_y = odomMsg->pose.pose.orientation.y;
  	float q_z = odomMsg->pose.pose.orientation.z;
  	float q_w = odomMsg->pose.pose.orientation.w;
	ang_z_ = atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_z*q_z + q_y*q_y));
}

void PosCorrect::corrCallBack(const std_msgs::Bool& checkerMsg){
	corr_checker=checkerMsg.data;
}

void PosCorrect::pidAlgorithm(){
	std_msgs::Float32 linear_error;
	std_msgs::Float32 angle_error;
	std_msgs::Float32 linear_velocity; // command
	std_msgs::Float32 angle_velocity; // command

    double Dx = pillar_x - pos_x_; // pos_x_ from odom (true sim. position)
    double Dy = pillar_y - pos_y_; // pos_y_ from odom (true sim. position)

	
	//(CUSTOM) Calculating distance from the pillar, and publishing it with dist_from_target_pub under the topic /dist_from_target
    std_msgs::Float32 dist_from_val;
	double distval=sqrt(std::pow(Dx,2)+std::pow(Dy,2));
	dist_from_val.data=distval;

	// update the pid status
	error_forward_prev_ = error_forward_;
	error_angle_prev_ = error_angle_;
	
	error_forward_ = sqrt(Dx*Dx + Dy*Dy) - target_distance;
	error_angle_ = atan2(Dy, Dx) - ang_z_;
    
	// regularize the error_angle_ within [-PI, PI]
	if(error_angle_ < -PI) error_angle_ += 2*PI;
	if(error_angle_ > PI) error_angle_ -= 2*PI;

	// integral term
	I_forward_ += dt * error_forward_;
	I_angle_ += dt * error_angle_;

	// derivative term
	D_forward_ = (-error_forward_prev_ + error_forward_) / dt;
	D_angle_ = (-error_angle_prev_ + error_angle_) / dt;

	// ENTER YOUR CODE HERE

	trans_angle_ = Kp_angle * error_angle_ + Ki_angle * I_angle_ + Kd_angle * D_angle_ ;
	trans_forward_=Kp_f * error_forward_ + Ki_f*I_forward_ + Kd_f*D_forward_ ;
	//trans_forward_*=200.0;

	// set limit
	int vel_lim;
	vel_lim=2; //change this as required
	if(trans_forward_ > vel_lim) trans_forward_ = vel_lim;
	if(trans_forward_ < -vel_lim) trans_forward_ = -vel_lim;

	// ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Distance: %f",  //#
	//	trans_forward_, trans_angle_, error_angle_, scan_range_);
 	ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Linear_error: %f",  
		trans_forward_, trans_angle_, error_angle_, error_forward_);

	double limit=0.01;
	if (distval<=limit){
		vel_cmd_.linear.x = 0;
		vel_cmd_.angular.z =0;
		vel_pub_.publish(vel_cmd_);
		kill=true;
		return;
	}
	//publish all
	vel_cmd_.linear.x = trans_forward_;
	vel_cmd_.angular.z = trans_angle_; //euler angle
	vel_pub_.publish(vel_cmd_);

	linear_error.data = error_forward_;
	error_forward_pub_.publish(linear_error);

	linear_velocity.data = trans_forward_;
	control_signal_forward_pub_.publish(linear_velocity);

	angle_error.data = error_angle_;
	error_angle_pub_.publish(angle_error);

	angle_velocity.data = trans_angle_;
	control_signal_angle_pub_.publish(angle_velocity);

	dist_from_target_pub_.publish(dist_from_val);

}

void PosCorrect::spin(){
	ros::Rate loop_rate(1/dt);	
	while(ros::ok()){
		ros::spinOnce();
		if (corr_checker==true){
			pidAlgorithm(); 
		}
		loop_rate.sleep();
		if (kill==true){
			return;
		}
	}

}

bool PosCorrect::loadParam(){

	//Bunch of errors that popup when the required public variables are not intialized in config.yaml
	if(!nodehandle_.getParam("/Kp_f", Kp_f)){
		ROS_ERROR("Kp_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_f", Ki_f)){
		ROS_ERROR("Ki_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kd_f", Kd_f)){
		ROS_ERROR("Kd_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_angle", Kp_angle)){
		ROS_ERROR("Kp_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_angle", Ki_angle)){
		ROS_ERROR("Ki_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kd_angle", Kd_angle)){
		ROS_ERROR("Kd_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_angle", target_angle)){
		ROS_ERROR("target_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_distance", target_distance)){
		ROS_ERROR("target_distance Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_x", pillar_x)){ //#
		ROS_ERROR("pillar_x Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_y", pillar_y)){ //#
		ROS_ERROR("pillar_y Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}
