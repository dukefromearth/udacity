#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	
	if(!client.call(srv))
		ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
// Saves the minumum x (min_white) and maximum x (max_white) values where a white pixel is present.
// Divides the min_white and max_white by 2 to find the center of the white image and checks if it is left/center/right.  
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
	int min_white = img.step+1;
	int max_white = -1;

	for(int i = 0; i < img.height; i++){
		for(int j = 0; j < img.step; j++){
			int red = i * img.step + j;
			if(img.data[red] == white_pixel && img.data[red+1] == white_pixel && img.data[red+2] == white_pixel){
				// Compare current value to see the minimum and maximum
				if(j < min_white) min_white = j;
				if(j > max_white) max_white = j;
			}
		}
	}

	// No white pixels seen on camera
	if(min_white > img.step || max_white < 0){
		drive_robot(0.0, 0.0);
		return;
	}
	
	// Center of the white object
	int mid_white = (max_white + min_white) / 2;

	// Divide the image into thirds and request action based on this.
	if(mid_white < img.step/3){
		drive_robot(0.1, 0.1);
	} else if (mid_white < 2 * img.step / 3) {
		drive_robot(0.5, 0.0);
	} else {
		drive_robot(0.0, -0.1);
	}
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
