#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "snap7.h"

#include <iostream>
using namespace std;

// plc的IP地址
const char* plc_ip = "192.168.1.33";

TS7Client snap7_client;

void callback(const geometry_msgs::Twist::ConstPtr& msg){
    int plc_speed[4];
    
    plc_speed[0] = (int)(27312.2626 * (msg->linear.x + msg->linear.y + 0.49 * msg->angular.z));
	plc_speed[1] = (int)(27312.2626 * (msg->linear.x - msg->linear.y - 0.49 * msg->angular.z));
	plc_speed[2] = (int)(27312.2626 * (msg->linear.x + msg->linear.y - 0.49 * msg->angular.z));
	plc_speed[3] = (int)(27312.2626 * (msg->linear.x - msg->linear.y + 0.49 * msg->angular.z));
    
    //cout << "right_front:" << plc_speed[0] << endl;
    byte buff[8] = { 0 };//创建一个读写缓存区
    for (int i = 0; i < 8; i++) 
    {
        if (i % 2) 
        {
            buff[i] = (byte)(0xff & (plc_speed[i / 2]));
        }
        else {
            buff[i] = (byte)(0xff & (plc_speed[i / 2] >> 8));
        }
    }
    
    // 向PLC写数据（参数分别是DB块，块号，起始地址， 写多少， 写word类型，数据源开始地址）
    snap7_client.AsWriteArea(0x84, 4, 0, 4, 0x04, buff);
}


int main(int argc, char *argv[])
{
    //plc connect
    snap7_client.ConnectTo(plc_ip, 0, 0); 

    if(!snap7_client.Connected())
    {
        cout<<"error return in "<<__FILE__<<" "<<__LINE__<<":erron type connect failed"<<endl;
        return -1;
    }else
    {
        cout<<"connect success"<<endl;
    }
   
    //ros
    ros::init(argc, argv, "cmd_to_plc");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, callback);
    ros::spin();
    return 0;
}