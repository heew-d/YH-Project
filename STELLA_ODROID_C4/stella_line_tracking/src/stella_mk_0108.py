#! /usr/bin/env python3

import time
import rospy
import cv2
import os
import sys, select, termios, tty

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from p_0103_function import test4_Import_lib

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
    settings = termios.tcgetattr(sys.stdin)
    angle_list = []

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    
    rospy.init_node('stella_line_tracking_node')
    bridge = CvBridge()
    # pub1 = rospy.Publisher('stella_line_tracking', Image, queue_size=10)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    
    try:
        while not rospy.is_shutdown():

            ret,frame = cap.read()
            
            if not ret:
                rospy.loginfo("Not Found Devices")
                break

            final_frame, angle = test4_Import_lib(frame)
            image_msg = bridge.cv2_to_imgmsg(final_frame,"bgr8")
            # cv2.imshow('final_frame', final_frame)
            
            if angle == -1.0:
                angle_list.append(angle)

            if len(angle_list) == 1:
                if target_linear_vel == target_linear_vel:
                   # target_linear_vel = 0.0
                    target_linear_vel = 0.3
                    target_angular_vel = 1*0.3 * -1.57
                    rospy.loginfo('Right')
                    angle_list.clear()
                        
            if -0.2 < angle < 0.5:
                if target_angular_vel == target_angular_vel:
                    target_linear_vel = 0.3
                    target_angular_vel = 0.0
                    rospy.loginfo('Straight')
            
                    
            # elif angle <= -1:     # back
            #     target_linear_vel = -1 * 0.3

            # elif angle >= 15 : # angle #left
                # target_angular_vel = 1 * 0.7 * 1.57

            # elif angle >=-15 and angle != -1 :   # angle #right
            #     target_angular_vel = 1 * 0.7 * -1.57

            # elif angle == -1 :
            #     target_linear_vel  = 0.0
            #     control_linear_vel = 0.0
            #     target_angular_vel = 0.0
            #     control_angular_vel = 0.0

            key = get_key()
            # print('key: ', key)
            # rospy.loginfo("Key", key)
            if key == '\x03':
            # if key == '\x1B':
                # rospy.loginfo("exit")

                cap.release()

                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)                
                # rospy.signal_shutdown(True)

                break

            # if rospy.is_shutdown():
            #     target_linear_vel  = 0.0
            #     target_angular_vel = 0.0
            twist.linear.x = target_linear_vel
            twist.angular.z = target_angular_vel
            
            # pub1.publish(image_msg)
            pub.publish(twist)
            rate.sleep()

        print('node exit')

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        cap.release()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
