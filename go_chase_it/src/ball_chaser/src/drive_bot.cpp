#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <string>

using namespace std;

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
	ROS_INFO("DriveToTarget service request received, starting...");
	geometry_msgs::Twist motor_command;
	motor_command.linear.x = req.linear_x;
	motor_command.angular.z = req.angular_z;
	motor_command_publisher.publish(motor_command);
	string linX = to_string((double)motor_command.linear.x);
	string angZ = to_string((double)motor_command.angular.z);
	res.msg_feedback = "Velocities were set to: linearx: " + linX +
	", angular_z; " + angZ;
	ROS_INFO_STREAM(res.msg_feedback);
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle n;
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::ServiceServer srv1 = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

	ros::spin();
	return 0;

}
