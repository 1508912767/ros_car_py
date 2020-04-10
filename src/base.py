#!/usr/bin/env python
# coding=utf-8

import rospy
from socket import *
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import tf
from nav_msgs.msg import Odometry
from threading import Thread
from math import *
import time

D = 0.095
car_width = 0.4


def callback(cmd_input, Socket):
    print("-----服务器已经启动成功，准备接收数据-----")
    Socket.settimeout(5)
    t = Twist()
    t.angular.z = cmd_input.angular.z
    t.linear.x = cmd_input.linear.x
    left_speed = t.linear.x - t.angular.z * 0.5 * 2
    right_speed = t.linear.x + t.angular.z * 0.5 * 2
    left_speed *= 1000
    right_speed *= 1000
    left_speed = str(left_speed)
    right_speed = str(right_speed)

    print("-----发送的速度数据-----")
    print("left_speed=%s" % left_speed)
    print("right_speed=%s" % right_speed)

    # Socket.send(b"左轮速度")
    Socket.send(left_speed.encode("utf-8"))
    # Socket.send(b"右轮速度")
    Socket.send(right_speed.encode("utf-8"))


def Send(Socket):
    rospy.Subscriber("cmd_vel", Twist, callback, Socket)
    rospy.Rate(5)
    rospy.spin()


def Receive(Socket):
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    rate = rospy.Rate(10)
    # 位姿
    x = 0.0
    y = 0.0
    th = 0.0
    el0 = 0
    er0 = 0
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        encode_data_left = Socket.recv(1024)
        encode_data_right = Socket.recv(1024)
        current_time = rospy.Time.now()

        if len(encode_data_left) and len(encode_data_right):
            # print("-----接收到编码器数据-----")
            # print("recvdata:%s" % encode_data_left)
            # print("recvdata:%s" % encode_data_right)
            el = int(encode_data_left.replace("\n", ""))
            er = int(encode_data_right.replace("\n", ""))

            dt = (current_time - last_time).to_sec()
            left_speed = 60.0 * ((el - el0) / (2400 * dt))
            right_speed = 60.0 * ((er - er0) / (2400 * dt))
            left_speed_ = left_speed * 3.14 * D / 60
            right_speed_ = right_speed * 3.14 * D / 60
            # print("左轮转速为: %f（r/min）, 右轮转速为：%f（r/min )" % (left_speed, right_speed))
            # print("左轮转速为: %f（m/s）,   右轮转速为：%f（m/s )" % (left_speed_, right_speed_))
            with open("./car_velocity", "w") as f:
                f.write(str(left_speed_) + "\n")
            with open("./car_velocity", "a") as f:
                f.write(str(right_speed_))

            # 逆解算之后, 将两轮速度转换成机器人速度
            vx = (left_speed_ + right_speed_) / 2
            vy = 0.0
            vth = (right_speed_ - left_speed_) / car_width

            # 获取机器人位态
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = -(vth * dt)
            # delta_x = vx * sin(th + vth/2)
            # delta_y = vy * cos(th + vth/2)
            # delta_th = vth
            x += delta_x
            y += delta_y
            th += delta_th

            odom_broadcaster = tf.TransformBroadcaster()
            odom_broadcaster.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(0, 0, th), current_time,
                                           "base_link", "odom")

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, th)

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth

            # publish the message
            odom_pub.publish(odom)

            # 更新编码器数据和时间
            el0 = el
            er0 = er
            last_time = current_time
            rate.sleep()

        else:
            print('-----未接收到客户端数据，可能连接已经断开-----')
            # Socket.send(b'client off')
            # 数据中断时进行服务重启程序，先close 再accept等待重新连线
            # 可以防止出现当client意外终止导致server的中断（Broken pipe错误）
            print('-----正在重新建立连接-----')
            Socket.close()
            Socket, clientInfo = serverSocket.accept()


if __name__ == "__main__":
    rospy.init_node("base")

    serverSocket = socket(AF_INET, SOCK_STREAM)
    serverSocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serverSocket.bind(('', 8899))
    serverSocket.listen(5)
    print("-----服务器正在启动-----")
    Sockets, clientInfo = serverSocket.accept()

    t1 = Thread(target=Receive, args=(Sockets,))
    t1.start()
    Send(Sockets)
