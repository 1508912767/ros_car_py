#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros_car_py/MsgCar.h>
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#define D 0.095
#define car_width 0.4

// 默认机器人的起始位置是odom参考系下的0点
//double x = 0.0;
//double y = 0.0;
//double th = 0.0;
double vx ;
double vy ;
double vth ;
//左右轮编码器(不同时间)
int el, er, el0, er0;
//左右轮速度(不同单位)
double left_speed, left_speed_, right_speed, right_speed_;


void enc_velCallback(const ros_car_py::MsgCar &encode)
{
    ros_car_py::MsgCar e = encode;
    //int encode_data_left, encode_data_right;

    el = e.encode_data_left;
    er = e.encode_data_right;

    //ROS_INFO("left_speed:%d   right_speed:%d",left_speed,right_speed);
}


//需要实现“odom”参考系到“base_link”参考系的变换，以及nav_msgs/Odometry消息的发布
int main(int argc, char** argv)
{
    //定义一个消息发布者来发布“odom”消息，再定义一个tf广播，来发布tf变换信息
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n,m;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

    //定义一个编码器数据接收者
    ros::Subscriber encode_sub = m.subscribe("MsgCar", 1, enc_velCallback);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    //初始化时间
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //使用10Hz的频率发布odom消息，在实际系统中，往往需要更快的速度进行发布
    ros::Rate r(10);

    while(n.ok())
    {
        //ROS_INFO("vx:%.3lf   vth:%.3lf",vx,vth);
        ros::spinOnce();
        current_time = ros::Time::now();
        //积分计算里程计信息
        double dt = (current_time - last_time).toSec();
        //current real speeds
        left_speed = 60.0*((el-el0)/(2400.0*dt));
        right_speed = 60.0*((er-er0)/(2400.0*dt));
        left_speed_ = left_speed*3.14*D/60;
        right_speed_ = right_speed*3.14*D/60;

        printf("左轮转速为: %f（r/min）, 右轮转速为：%f（r/min )\n", left_speed, right_speed);
        printf("左轮转速为: %f（m/s）,   右轮转速为：%f（m/s )\n", left_speed_, right_speed_);

        //逆解算之后,将两轮速度转换成机器人速度
        vx = (left_speed_ + right_speed_)/2;
        vy = 0.0;
        vth = (right_speed_ - left_speed_)/car_width;

        //获取机器人位态
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        x += delta_x;
        y += delta_y;
        th += delta_th;

        //为了兼容二维和三维的功能包，让消息结构更加通用，里程计的偏航角需要转换成四元数才能发布，辛运的是，ROS为我们提供了偏航角与四元数相互转换的功能
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        //更新编码器数据
        last_time = current_time;
        el0 = el;
        er0 = er;
        r.sleep();

    }
}



quaternion = Quaternion()
quaternion.x = 0.0
quaternion.y = 0.0
quaternion.z = sin(self.th / 2.0)
quaternion.w = cos(self.th / 2.0)

self.odomBroadcaster.sendTransform(
(self.x, self.y, 0),
(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
rospy.Time.now(),
self.base_frame, // 子坐标系
"odom" // 父坐标系
)

