#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include "myutil.hpp"


using namespace std;


int robot_mode;

double speed[3];

geometry_msgs::Twist cmd_msg;

ros::Publisher pub;

cv_bridge::CvImagePtr cv_ptr;


void callback( const sensor_msgs::Image::ConstPtr& msg_rgb ){

    if(ros::param::get("robot_state",robot_mode) == false) robot_mode = 0;

    if(robot_mode == 0) return;
      
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    } 

    followlane(cv_ptr->image, speed);

    cmd_msg.linear.x = speed[0];
    cmd_msg.linear.y = speed[1];
    cmd_msg.angular.z = speed[2];

    //ROS_INFO("speed:%f", speed[1]);
    pub.publish(cmd_msg);

}

int main(int argc, char** argv){

    robot_mode = -1;

    //ros
    ros::init(argc, argv, "follow_lane");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect_color", 1, &callback);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",  10);

    ros::spin();

    return 0;
}