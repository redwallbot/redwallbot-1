#include "myjoystick.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

bool joy_flag;

double speed[3];
int c_mode;

void* joyThread(void *);

pthread_mutex_t s_mutex;

int main(int argc, char **argv)
{

    geometry_msgs::Twist msg;
    
    joy_flag = true;

    memset(speed, 0, sizeof(speed));

    c_mode = -1;

    pthread_t pth_joy;
    pthread_create(&pth_joy,  NULL, joyThread, NULL);
    pthread_detach(pth_joy);

    ros::init(argc, argv, "joystick_pub");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",  10);
    ros::Rate loop_rate(100);


    while(ros::ok())
    {
        if(c_mode == 1)
        {
            ros::param::set("robot_state",  1);
            continue;
        }

        ros::param::set("robot_state",  0);

        pthread_mutex_lock(&s_mutex);
        msg.linear.x = speed[0];
        msg.linear.y = speed[1];
        msg.angular.z = speed[2];
        pthread_mutex_unlock(&s_mutex);
        
       
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    joy_flag = false;
    usleep(500);
    pthread_mutex_destroy(&s_mutex);
    return 0;
}


void* joyThread(void *)
{
    MyJoyStick joy;
    if(!joy.initJostick())
    {
        cout << "init joystick faild" << endl;
        return 0;
    }

    while(joy_flag)
    {
         joy.listenJs();
         pthread_mutex_lock(&s_mutex);
        joy.getSpeed(speed);
        c_mode = joy.getMode();
        pthread_mutex_unlock(&s_mutex);
    }
    return 0;
}