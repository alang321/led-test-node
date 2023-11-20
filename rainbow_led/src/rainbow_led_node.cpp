#include <ros/ros.h>
#include <string>
#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>
#include <cmath>

void hsvToRgb(float h, float s, float v, int rgb[3]);

//main function for the opencv person detector node, which creates the opencv person detector object and spinnnnns
int main(int argc, char **argv)
{
    //led_msgs::SetLEDs::Request&
    ros::init(argc, argv, "rainbow_led_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting rainbow_led_node.");

    ros::service::waitForService("/led/set_leds");
    ROS_INFO("Service found.");
    
    // Create a ROS service client
    ros::ServiceClient client = nh.serviceClient<led_msgs::SetLEDs>("/led/set_leds");

    // Create a service message
    led_msgs::SetLEDs srv;



    int num_leds = 2;
    

    // Set the initial iteration value
    int iteration = 0;
    float duration = 7.0f;
    int rate_hz = 10;

    // Loop at 20Hz until the node is shut down
    ros::Rate rate(rate_hz);

    while(ros::ok()) {
        // Clear the LED array for each iteration
        srv.request.leds.clear();

        for (int i = 0; i < num_leds; ++i)
        {
            led_msgs::LEDState led_msg;
            led_msg.index = i;

            // Calculate HSV values for rainbow effect
    	    float normalized_iteration = static_cast<float>(iteration) / (rate_hz * duration);
            ROS_INFO("Normalized Iteration: %.2f", normalized_iteration);

            float hue = fmod(normalized_iteration + i / static_cast<float>(num_leds), 1.0);
            ROS_INFO("Hue for LED %d: %.2f", i, hue);

            int rgb[3];
            hsvToRgb(hue, 1.0, 1.0, rgb);
		
            led_msg.r = rgb[0];
            led_msg.g = rgb[1];
            led_msg.b = rgb[2];
            ROS_INFO("LED %d - RGB: %d, %d, %d", i, rgb[0], rgb[1], rgb[2]);
            srv.request.leds.push_back(led_msg);
        }

        // Call the service
        if (!client.call(srv))
        {
            ROS_ERROR("Failed to call service set_leds");
        }
        
        // Increment the iteration for the next cycle
        iteration++;

        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}


void hsvToRgb(float h, float s, float v, int rgb[3])
{
    int i;
    float f, p, q, t;

    if (s == 0)
    {
        // achromatic (grey)
        rgb[0] = rgb[1] = rgb[2] = static_cast<int>(v * 255);
        return;
    }

    h /= 60;            // sector 0 to 5
    i = static_cast<int>(std::floor(h));
    f = h - i;          // factorial part of h
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));

    switch (i)
    {
    case 0:
        rgb[0] = static_cast<int>(v * 255);
        rgb[1] = static_cast<int>(t * 255);
        rgb[2] = static_cast<int>(p * 255);
        break;
    case 1:
        rgb[0] = static_cast<int>(q * 255);
        rgb[1] = static_cast<int>(v * 255);
        rgb[2] = static_cast<int>(p * 255);
        break;
    case 2:
        rgb[0] = static_cast<int>(p * 255);
        rgb[1] = static_cast<int>(v * 255);
        rgb[2] = static_cast<int>(t * 255);
        break;
    case 3:
        rgb[0] = static_cast<int>(p * 255);
        rgb[1] = static_cast<int>(q * 255);
        rgb[2] = static_cast<int>(v * 255);
        break;
    case 4:
        rgb[0] = static_cast<int>(t * 255);
        rgb[1] = static_cast<int>(p * 255);
        rgb[2] = static_cast<int>(v * 255);
        break;
    default:    // case 5:
        rgb[0] = static_cast<int>(v * 255);
        rgb[1] = static_cast<int>(p * 255);
        rgb[2] = static_cast<int>(q * 255);
        break;
    }
}
