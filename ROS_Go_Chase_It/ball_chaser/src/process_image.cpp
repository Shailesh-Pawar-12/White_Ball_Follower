#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"      // Service                                   
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{


    ROS_INFO_STREAM("Moving the bot with requested velocities");

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int found_position = -1;
    int white_pixel_thresh = 200; // its giving good results with value > 200

    for (int i = 0; i < img.height *img.step; i=i+3)   
    {
	int red=img.data[i];
        int green=img.data[i+1];
        int blue=img.data[i+2];
	if (red> white_pixel_thresh && green> white_pixel_thresh && blue> white_pixel_thresh) {
           found_position = i ;                                                                 // White ball found at position i
	   break; 
        }
    }
    if (found_position != -1)
    {    
	int remainder = found_position%img.step;
	float pixel_location = remainder / 3; 

    	if (pixel_location >= img.width / 3  && pixel_location < 2*img.width /3 )
    	{
		drive_robot(0.1,0);                                                   // Move Forward
    	}
    	else if (pixel_location >= 2*img.width /3 && pixel_location < img.width)
    	{
		drive_robot(0,-0.1);                                                  // Move Left
    	}
    	else if (pixel_location >= 0 && pixel_location < img.width / 3)
    	{
		drive_robot(0,0.1);                                                   // Move Right 
    	}
     }
     else{
        	drive_robot(0,0);                                                     // Stop
	}
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    ROS_INFO("img.data[");
    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
