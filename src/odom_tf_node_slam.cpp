#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include "string.h"

using namespace std;
using namespace boost::asio;

double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double dt = 0.0;


//Defines the packet protocol for communication between upper and lower computers
#pragma pack(1)
typedef union _Upload_speed_
{
    unsigned char buffer[16];
    struct _Speed_data_
    {
        float Header;
        float X_speed;
        float Y_speed;
        float Z_speed;
    }Upload_Speed;
}Struct_Union;
#pragma pack(4)

//Initializes the protocol packet data
Struct_Union Reciver_data        = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Struct_Union Transmission_data   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Defines the message type to be transmitted geometry_msgs
geometry_msgs::Quaternion odom_quat;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    geometry_msgs::Twist twist = twist_aux;

    Transmission_data.Upload_Speed.Header = header_data;
    Transmission_data.Upload_Speed.X_speed = twist_aux.linear.x;
    Transmission_data.Upload_Speed.Y_speed = twist_aux.linear.y;
    Transmission_data.Upload_Speed.Z_speed = twist_aux.angular.z;
}


int main(int argc, char** argv)
{
    unsigned char check_buf[1];
    std::string usart_port,robot_frame_id,smoother_cmd_vel;
    int baud_data;
    float p,i,d;

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;
    ros::Time current_time, last_time;

    //Gets the parameters in the launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("usart_port", usart_port, "/dev/robot_base");
    nh_private.param<int>("baud_data", baud_data, 115200);
    nh_private.param<std::string>("robot_frame_id", robot_frame_id, "base_link");
    nh_private.param<std::string>("smoother_cmd_vel", smoother_cmd_vel, "/cmd_vel");


    //Create a boot node for the underlying driver layer of the robot base_controller
    ros::Subscriber cmd_vel_sub = n.subscribe(smoother_cmd_vel, 50, cmd_velCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    //Initializes the data related to the boost serial port
    io_service iosev;
    serial_port sp(iosev, usart_port);
    sp.set_option(serial_port::baud_rate(baud_data));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    while(ros::ok())
    {
        ros::spinOnce();
        //Gets the cycle of two time slice rotations. The purpose is to calculate the odom message data
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        last_time = ros::Time::now();

        //Read the data from the lower computer
        read(sp, buffer(Reciver_data.buffer));
        if(Reciver_data.Upload_Speed.Header > 15.4 && Reciver_data.Upload_Speed.Header < 15.6)
        {
                vx  = Reciver_data.Upload_Speed.X_speed;
                vy  = Reciver_data.Upload_Speed.Y_speed;
                vth = Reciver_data.Upload_Speed.Z_speed;
                //ROS_INFO("%2f    %2f    %2f",vx,vy,vth);
        }
        else
        {
            //ROS_INFO("------Please wait while the robot is connected!-----");
            read(sp, buffer(check_buf));
        }
        //Send the next bit machine under the cmd_val topic to monitor the speed information
        write(sp, buffer(Transmission_data.buffer,16));
        //Calculate odometer data
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = robot_frame_id;

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;


        odom.child_frame_id = robot_frame_id;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        //Release odometer data between odom-> base_link
        odom_pub.publish(odom);
    }
    iosev.run();
}