#ifndef MYJOYSTICK_HPP
#define MYJOYSTICK_HPP

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <netdb.h>
#include <ctime>
#include <stdarg.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <linux/input.h>  
#include <linux/joystick.h> 

using namespace std;


#define AXES_LLR       0x00       // 左遥杆x轴
#define AXES_LUD       0x01       // 左摇杆y轴 
#define AXES_RLR       0x02       // 右摇杆x轴   
#define AXES_RUD       0x03       // 右遥杆y轴

#define BUTTON_X       0x00                        
#define BUTTON_A       0x01                   
#define BUTTON_B       0x02                   
#define BUTTON_Y       0x03
#define BUTTON_BACK    0x08                   
#define BUTTON_START   0x09  


const double V_MAXSPEED = 0.6;

const double W_MAXSPEED = 1.93;

class MyJoyStick
{
private:
    int m_js_fd;
    int m_mode;
    struct js_event m_js;
    int m_len, m_joy_type, m_joy_num, m_joy_value ;

    double m_speed[3];

public:
    bool initJostick()
    {
        if(m_js_fd > 0)
        {
            close(m_js_fd);
            m_js_fd = -1;
        } 
        m_len = m_joy_type = m_joy_num = m_joy_value = 0;
        m_mode = -1;
        
        memset(m_speed, 0, sizeof(m_speed));
        memset(&m_js, 0, sizeof(js_event));
        
        if ((m_js_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK )) < 0)
        {  
            cout << "joystick connected failed" << endl;  
            return false;  
         }
        return true;  
    }

    bool listenJs()
    {        
        fd_set rset;
        struct timeval time_out;
        FD_ZERO(&rset);
        time_out.tv_sec = 0;
        time_out.tv_usec = 0;
        FD_SET(m_js_fd, &rset);

        if(select(m_js_fd + 1, &rset, 0, 0, &time_out) < 0)
        {
            perror("ERR:read serial ");
            return false;
        }
        m_len = read(m_js_fd, &m_js, sizeof(struct js_event));  
        
        m_joy_value = m_js.value;
        m_joy_type = m_js.type;
        m_joy_num = m_js.number;

        if(m_joy_value > -600.0 && m_joy_value < 600.0){
            m_joy_value = 0.0;
        }

        if (m_joy_type == JS_EVENT_BUTTON)  {  
            if( m_joy_num == BUTTON_X  ){
                cout << "button_X" << endl;  
                
            }
            if( m_joy_num == BUTTON_B  ){
                cout << "button_B" << endl; 
                m_mode = 0; 

            }
            if( m_joy_num == BUTTON_A  ){
                cout << "button_A" << endl;
                m_mode = 1;  
 
            }
            if( m_joy_num == BUTTON_Y  ){
                cout << "button_Y" << endl; 
            }
            if(m_joy_num == BUTTON_BACK){
                cout << "button_back" << endl;
                m_mode = -1;
                
            }
            if(m_joy_num == BUTTON_START){
                cout << "button_start" << endl;
                m_mode = 0;
                
            }
        }else if (m_joy_type == JS_EVENT_AXIS ){

            if(m_joy_num == AXES_LUD ){
                m_speed[0] = - V_MAXSPEED * (m_joy_value / 32767.0);

            }
            
            if(m_joy_num == AXES_LLR){
                m_speed[1] = - V_MAXSPEED * (m_joy_value / 32767.0);

            }
            
            if(m_joy_num == AXES_RLR){
                m_speed[2] = - W_MAXSPEED * (m_joy_value / 32767.0);

            }
            //cout << m_speed[0] << "\t" << m_speed[1] << "\t" << m_speed[2] << endl;

        }

        if(m_mode == -1){
            memset(m_speed, 0, sizeof(m_speed));
        }
        
        return true;

    }

    void getSpeed(double *speed){
        memcpy(speed, m_speed, sizeof(m_speed));
    }

    int getMode(){
        return m_mode;
    }

    ~MyJoyStick(){
        if(m_js_fd > 0){
            close(m_js_fd);
            m_js_fd = -1;
        }
    }

};

#endif