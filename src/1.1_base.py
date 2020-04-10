#!/usr/bin/env python
# coding=utf-8

import rospy
from socket import *
from geometry_msgs.msg import Twist
from ros_car_py.msg import MsgCar
from threading import Thread
import time


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
    pub = rospy.Publisher('MsgCar', MsgCar, queue_size=1)
    Encode = MsgCar()
    rospy.Rate(5)
    while True:
        encode_data_left = Socket.recv(1024)
        encode_data_right = Socket.recv(1024)

        if len(encode_data_left) and len(encode_data_right):
            # print("-----接收到编码器数据-----")
            # print("recvdata:%s" % encode_data_left)
            # print("recvdata:%s" % encode_data_right)
            Encode.encode_data_left = int(encode_data_left.replace("\n",""))
            Encode.encode_data_right = int(encode_data_right.replace("\n",""))
            pub.publish(Encode)

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
    # t2 = Thread(target=Send, args=(Sockets,))
    t1.start()
    #t2.start()
    # t1.join()
    # t2.join()
    Send(Sockets)

