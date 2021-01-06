#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>
#include "std_msgs/String.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include <sstream>


class myrobotTeleop{
public:
	myrobotTeleop();
private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
	bool IsZeroMsg(geometry_msgs::Twist cmd_vel);
	void publish();
	

	ros::NodeHandle ph_, nh_, fh_;

	int axis_left_lr_joy_,
	axis_left_ud_joy_,
	axis_right_lr_joy_,
	axis_right_ud_joy_,
	axis_right_throttle_,
	axis_left_throttle_,
	button_x_,
	button_y_,
	button_a_,
	button_b_,
	button_LB_,
	button_RB_;
	double max_speed_, max_omega_;
	double linear_scale_, angular_scale_; // for diff drive
	double y_dot_scale_, x_dot_scale_; // for omni drive
	bool new_publish_;
	bool zero_twist_published_;
	ros::Publisher vel_pub_;
	ros::Publisher nav_abort_pub_;
	ros::Subscriber joy_sub_;
	ros::Subscriber goal_sub_;
	ros::Publisher goal_pub_;
	

	geometry_msgs::Twist last_published_;
	geometry_msgs::PoseStamped current_goal;
	boost::mutex publish_mutex_;
	ros::Timer timer_;

};

myrobotTeleop::myrobotTeleop():
				ph_("~"),
				axis_left_lr_joy_(0),
				axis_left_ud_joy_(1),
				axis_right_lr_joy_(3),
				axis_right_ud_joy_(4),
				axis_right_throttle_(5),
				axis_left_throttle_(2),
				button_x_(2),
				button_y_(3),
				button_a_(0),
				button_b_(1),
				button_LB_(4),
				button_RB_(5),
				max_speed_(0.5),
				max_omega_(0.5)
{
	ph_.param("axis_left_lr_joy", axis_left_lr_joy_, axis_left_lr_joy_);
	ph_.param("axis_left_ud_joy", axis_left_ud_joy_, axis_left_ud_joy_);
	ph_.param("axis_right_lr_joy", axis_right_lr_joy_, axis_right_lr_joy_);
	ph_.param("axis_right_ud_joy", axis_right_ud_joy_, axis_right_ud_joy_);
	ph_.param("axis_right_throttle", axis_right_throttle_, axis_right_throttle_);
	ph_.param("axis_left_throttle", axis_left_throttle_, axis_left_throttle_);

	ph_.param("button_x", button_x_, button_x_);
	ph_.param("button_y", button_y_, button_y_);
	ph_.param("button_a", button_a_, button_a_);
	ph_.param("button_b", button_b_, button_b_);
	ph_.param("button_LB", button_LB_, button_LB_);
	ph_.param("button_RB", button_RB_, button_RB_);

	ph_.param("max_speed", max_speed_, max_speed_);
	ph_.param("max_omega", max_omega_, max_omega_);

	linear_scale_ = max_speed_ / 2.0;
	angular_scale_ = max_omega_;

	y_dot_scale_ = max_speed_;
	x_dot_scale_ = max_speed_;

	new_publish_ = false;
	zero_twist_published_ = false;

	vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	nav_abort_pub_ = ph_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1, true);
	goal_pub_ = ph_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1, true);
	
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &myrobotTeleop::joyCallback, this);	
	goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base/current_goal", 10, &myrobotTeleop::goalCallback, this); 

	timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&myrobotTeleop::publish, this));
	
}

void myrobotTeleop::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
	
	current_goal.pose.position = goal->pose.position;
	current_goal.pose.orientation = goal->pose.orientation;
	//ROS_INFO_STREAM("Current Goal: "<<current_goal.pose);
	
}

void myrobotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	geometry_msgs::Twist vel;
	actionlib_msgs::GoalID abort; 
	static move_base_msgs::MoveBaseGoal resume_goal; 
	move_base_msgs::MoveBaseActionGoal resume_action_goal;

	if (joy->buttons[button_x_] == 1)
	{
	  abort.stamp = ros::Time::now();
	  abort.id = "";
	  nav_abort_pub_.publish(abort);
	}
	
	
	if (joy->buttons[button_a_] == 1)
	{ 
	  resume_goal.target_pose.header.stamp = ros::Time::now();
	  resume_goal.target_pose.pose.position = current_goal.pose.position; 
	  resume_goal.target_pose.pose.orientation = current_goal.pose.orientation;
	  
	  //ROS_INFO_STREAM("resume goal:"<<resume_goal.target_pose.pose);
	  
	  abort.stamp = ros::Time::now();
	  abort.id = "";
	  nav_abort_pub_.publish(abort);  
	  
	}
	
	if (joy->buttons[button_b_] == 1)
	{
	  //ROS_INFO_STREAM("resume goal:"<<resume_goal.target_pose.pose);
	  resume_action_goal.header.stamp = ros::Time::now();
	  resume_action_goal.goal.target_pose.header.frame_id = "map";
	  resume_action_goal.goal.target_pose.pose = resume_goal.target_pose.pose; 
	  goal_pub_.publish(resume_action_goal);
 
	}

	if(joy->axes[axis_right_lr_joy_]!=0 || joy->axes[axis_right_ud_joy_]!=0){

			vel.angular.z = angular_scale_ * joy->axes[axis_left_lr_joy_];

			vel.linear.x = linear_scale_ * (joy->axes[axis_left_throttle_]-joy->axes[axis_right_throttle_]);
			vel.linear.y = -y_dot_scale_ * (-joy->axes[axis_right_lr_joy_]);
		}
		else{
			vel.angular.z = angular_scale_ * joy->axes[axis_left_lr_joy_];
			vel.linear.x = linear_scale_ * (joy->axes[axis_left_throttle_]-joy->axes[axis_right_throttle_]);
		}

	last_published_ = vel;
	new_publish_ = true;
}

void myrobotTeleop::publish(){
	boost::mutex::scoped_lock lock(publish_mutex_);
	if(new_publish_){
		bool is_zero_twist = IsZeroMsg(last_published_);

		if(!(is_zero_twist && zero_twist_published_)){
			vel_pub_.publish(last_published_);
		}

		zero_twist_published_ = is_zero_twist;
		new_publish_ = false;
	}
}

bool myrobotTeleop::IsZeroMsg(geometry_msgs::Twist cmd_vel)
{
  bool result = true;
  if( (cmd_vel.linear.x) != 0 || (cmd_vel.linear.y != 0) || (cmd_vel.angular.z != 0) )
  {
    result = false;
  }

  return result;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "myrobot_teleop");
	myrobotTeleop myrobot_teleop;

	ros::spin();
}
