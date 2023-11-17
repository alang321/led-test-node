#include <ros/ros.h>
#include <string>
#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>

//main function for the opencv person detector node, which creates the opencv person detector object and spinnnnns
int main(int argc, char **argv)
{
    //led_msgs::SetLEDs::Request&
    ros::init(argc, argv, "led_test_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting led_test_node.");

    ros::service::waitForService("set_leds");
    ROS_INFO("Service found.");
    
    // Create a ROS service client
    ros::ServiceClient client = nh.serviceClient<led_msgs::SetLEDs>("set_leds");

    // Create a service message
    led_msgs::SetLEDs srv;



    // Fill in the LED data. This is just an example; you should replace it with your actual LED data.
    led_msgs::LEDState led_msg1;
    led_msg1.index = 0;
    led_msg1.r = 0;
    led_msg1.g = 255;
    led_msg1.b = 0;
    srv.request.leds.push_back(led_msg1);
    
    
    // Fill in the LED data. This is just an example; you should replace it with your actual LED data.
    led_msgs::LEDState led_msg2;
    led_msg2.index = 1;
    led_msg2.r = 255;
    led_msg2.g = 0;
    led_msg2.b = 0;
    srv.request.leds.push_back(led_msg2);
    
    // Call the service
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("LEDs set successfully");
        }
        else
        {
            ROS_ERROR("Failed to set LEDs: %s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service set_leds");
        return 1;
    }
    
    return 0;
}
