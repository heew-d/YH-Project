#! /usr/bin/env python3

import socket
from _thread import *


import time
import rospy
import cv2
import os

import base64
import numpy as np

import sys, select, termios, tty

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from ji_func_0109 import Orchestrator

# 쓰레드에서 실행되는 코드입니다. 
# 접속한 클라이언트마다 새로운 쓰레드가 생성되어 통신을 하게 됩니다. 
def threaded(client_socket, addr): 

    global clients

    print('Connected by :', addr[0], ':', addr[1]) 

    clients.append(client_socket)
    clientIndex = len(clients) - 1

    # 클라이언트가 접속을 끊을 때 까지 반복합니다. 
    while True: 

        try:
            # 데이터가 수신되면 클라이언트에 다시 전송합니다.(에코)
            data = client_socket.recv(1024)
            
            if not data: 
                print('Disconnected by ' + addr[0],':',addr[1])
                clients.pop(clientIndex)

                break

            print('Received from ' + addr[0],':',addr[1] , data.decode())

            client_socket.send(data) 

        except ConnectionResetError as e:

            print('Disconnected by ' + addr[0],':',addr[1])
            break
             
    client_socket.close() 


# HOST = '127.0.0.1'
HOST = '192.168.0.19'
PORT = 9999

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT)) 
server_socket.listen() 

print('server start')


clients = []

def sendCamThread(cap):

    global clients
    global target_linear_vel
    global target_angular_vel
    bridge = CvBridge()

    while cap.isOpened():
        retval, frame = cap.read()

        # if not retval:
        #     continue

        if not retval:
            rospy.loginfo("Not Found Devices")

            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            pub2.publish(twist)

            break

        final_frame, angle = Orchestrator(frame)
        image_msg = bridge.cv2_to_imgmsg(final_frame,"bgr8")
        # cv2.imshow('final_frame', final_frame)
        print('angle: ',angle)

        # save frame to file..
        out.write(final_frame)
        
        
        if angle == -1:
            angle_list.append(angle)

        if len(angle_list) == 1:
            if target_linear_vel == target_linear_vel:
                target_linear_vel = 0.0
                # target_linear_vel = 0.2
                target_angular_vel = 0.3 * -1.57
                # rospy.loginfo('Right')
                angle_list.clear()
                    
        if -0.2 < angle < 0.5:
            if target_angular_vel == target_angular_vel:
                target_angular_vel = 0.0
                target_linear_vel = 0.3

        twist.linear.x = target_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = target_angular_vel

        pub1.publish(image_msg)
        pub2.publish(twist)

        key = get_key()

        if key == '\x03': 
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            pub2.publish(twist)
            break

        cv2.waitKey(40) #33ms == 30fps        

        decoImg = cv2.imencode('.jpg', frame)[1]
        # bData = decoImg.tobytes()
        bData = base64.b64encode(decoImg)

        # print('imageBytes: ', len(imageBytes))
        
        for client in clients:
            # byte로 바꿔서 전송
            # client.send('sendframe'.encode())
            try:
                client.send(bData)
                # client.send(' '.encode())
                client.send(b'\x20')
            except:
                pass

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    angle_list = []

    cap = cv2.VideoCapture(0)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,120)
    start_new_thread(sendCamThread, (cap,))

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    # fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
    out = cv2.VideoWriter('/home/odroid/catkin_ws/src/STELLA_ODROID_C4/stella_line_tracking/src/SaveVideo.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width, frame_height))

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    
    rospy.init_node('stella_line_tracking_node')
    
    pub1 = rospy.Publisher('stella_line_tracking', Image, queue_size=1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    
    try:
        
        while not rospy.is_shutdown():
            print('wait')

            

            # rospy.loginfo("run")
            # print('run')

            settings = termios.tcgetattr(sys.stdin)

            twist = Twist()
            client_socket, addr = server_socket.accept()
            start_new_thread(threaded, (client_socket, addr)) 

            # begin = time.time()

            # ret,frame = cap.read()
            
            
        server_socket.close()

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        rospy.loginfo("Finally")
        cap.release()
        out.release()
        server_socket.close()
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)