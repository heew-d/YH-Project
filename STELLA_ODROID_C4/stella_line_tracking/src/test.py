#! /usr/bin/env python3

import time
import rospy
import cv2
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from p_0103_function import test4_Import_lib

# print(test4_Import_lib)

# def weight_moving_average(w1,w2,w3,w4):   #w1= 0.5 //  w2 = 0.3 // w3 = 0.15 // w4 = 0.05
#     global prev_angle
#     if len(prev_angle) <4:
#         WMA_angle = prev_angle[0]
#         # pass
#     elif len(prev_angle)>=4:    
#         WMA_angle = prev_angle[0]w4+prev_angle[1]w3+prev_angle[2]w2+prev_angle[3]w1 
        
#         prev_angle.pop(0)
#         # print('WMA_angle: ',WMA_angle)
#     return WMA_angle


if __name__=="__main__":
    empty_list = []

    cap = cv2.VideoCapture(0)
    # cap = cv2.VideoCapture('/home/odroid/catkin_ws/src/STELLA_ODROID_C4/data/trim.MOV')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    # status, image = cap.read()

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    
    rospy.init_node('stella_line_tracking_node')
    bridge = CvBridge()
    pub1 = rospy.Publisher('stella_line_tracking', Image, queue_size=1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    
    try:
        
        while not rospy.is_shutdown():
            ref,frame = cap.read()
            if not ref:
                rospy.loginfo("Not Found Devices")
                break

            # print('frame:',frame.shape)
            # cv2.imshow('frame', frame)

            # print(frame.shape)
            #frame = cv2.flip(frame,0)
            #frame = cv2.flip(frame,1)
            
            # if cv2.waityKey(1) == 27:
            # if key == 27:    
            #     target_linear_vel  = 0.0
            #     control_linear_vel = 0.0
            #     target_angular_vel = 0.0
            #     control_angular_vel = 0.0            
            #     break
            resized = cv2.resize(frame, (160,120))
            final_frame, angle = test4_Import_lib(resized)    # Process each frames
            
            
            # print('angle: ', round(angle, 1))
            # print(final_frame.shape)

            image_msg = bridge.cv2_to_imgmsg(final_frame,"bgr8")
            # print(target_linear_vel)
            # print(target_angular_vel)
            print('angle: ',angle)
            # cv2.imshow('final_frame', final_frame)
        
            # rate.sleep()
            # cv2.waitKey(1)
            if angle >= 0.1 and angle <= 1.0:    #straight
                target_linear_vel = 0.3          #속도값(0.1~1.0까지 수가 커질수록 속도 up)
                target_angular_vel = 0.0
                print("straight")

            # elif angle <= -1:                     # back
            #     target_linear_vel = -0.3          # 안쓰니깐 주석처리

            # elif angle >= 15 :                    # angle #left
            #     target_angular_vel = 0.3 * 1.57   #1.57 뜻?

            elif angle >1.0 and angle <4.0:      #right
                target_angular_vel = 0.3 * -1.57
                target_linear_vel = 0.2
                #test

            elif angle == -20:                    #stop
                # target_linear_vel  = 0.0
                # control_linear_vel = 0.0
                # target_angular_vel = 0.0
                # control_angular_vel = 0.0
                empty_list.append(angle)
                if len(empty_list) == 15:
                    
                    target_angular_vel = 0.5 * -1.57
                    target_linear_vel = 0.2
                    empty_list.clear()
                  

                


            # if rospy.is_shutdown():
            #     target_linear_vel  = 0.0
            #     control_linear_vel = 0.0
            #     target_angular_vel = 0.0
            #     control_angular_vel = 0.0
            key = cv2.waitKey(1)
            twist = Twist()

            twist.linear.x = target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel

            # cv2.imshow('final_frame', final_frame)
            pub1.publish(image_msg)
            pub2.publish(twist)

            

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        cap.release()
        pass