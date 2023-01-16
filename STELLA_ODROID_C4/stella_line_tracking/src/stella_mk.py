#! /usr/bin/env python3

import time
import rospy
import cv2
import os

import sys, select, termios, tty

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from ji_func_0109 import Orchestrator


bridge = None
pub1 = None
pub2 = None
settings = None
twist = Twist()


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def testDrive():
    rospy.loginfo("testDrive")

    rate = rospy.Rate(1)

    for _ in range(3):

        publishTwist(0.2, 0.3)

        rate.sleep()
        pass

    publishTwist()


def testCam():

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,120)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    print('cap: frame_width:', frame_width)
    print('cap: frame_height:', frame_height)

    while True:
        ret, frame = cap.read()
        print('ret: ', ret)

        if not ret:
            break
        pass

    pass

def drive():

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,120)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    # fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
    # out = cv2.VideoWriter('/home/odroid/catkin_ws/src/STELLA_ODROID_C4/stella_line_tracking/src/SaveVideo.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width, frame_height))

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    
    rate = rospy.Rate(60)
    
    try:
        while not rospy.is_shutdown():
            # rospy.loginfo("run")
            # print('run')

            # begin = time.time()

            key = get_key()
            # print('key: ', key)
            # rospy.loginfo("Key", key)
            if key == '\x03': 
                rospy.loginfo('force shutdown')
                
                publishTwist()
                break


            ret, final_frame, angle = lineTracking(cap)
            if not ret:
                rospy.loginfo("Not Found Devices")

                publishTwist()
                break

            # ret,frame = cap.read()
            # if not ret:
            #     rospy.loginfo("Not Found Devices")

            #     publishTwist()
            #     break

            # final_frame, angle = Orchestrator(frame)



            image_msg = bridge.cv2_to_imgmsg(final_frame,"bgr8")
            # cv2.imshow('final_frame', final_frame)
            print(f'angle: {angle}')
            # rospy.loginfo(f"angle: {angle}")

            # save frame to file..
            # out.write(final_frame)
            
            
                        
            if angle == 0:
                # if target_angular_vel == target_angular_vel:
                target_linear_vel = 0.1
                target_angular_vel = 0.0
                # rospy.loginfo('Straight')

            

            # elif angle > 0.5 and angle != -1 :   # angle #right
            #     target_angular_vel = 0.3 * -1.57
            #     target_linear_vel =  0.1
            
            # elif angle == -1 :
            #     target_linear_vel  = 0.0
            #     control_linear_vel = 0.0
            #     target_angular_vel = 0.0
            #     control_angular_vel = 0.0




            # delta = time.time() - begin
            # print("delta: ", delta)

            # key = get_key()
            # # print('key: ', key)
            # # rospy.loginfo("Key", key)
            # if key == '\x03': 
            #     rospy.loginfo('force shutdown')
                
            #     publishTwist()
            #     break


            publishTwist(target_linear_vel, target_angular_vel)
            pub1.publish(image_msg)

            # cv2.waitKey(40) #33ms == 30fps
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        rospy.loginfo("Finally")
        cap.release()
        # out.release()
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def lineTracking(cap):

    datas = []
    now = time.time()

    while True:

        ret, frame = cap.read()
        if not ret:
            continue

        final_frame, angle = Orchestrator(frame)

        datas.append(angle)
        deltaSec = time.time() - now
        if deltaSec >= 1.0:
            break

        time.sleep(0.05) #20fps
        
    # for _ in range(20):
    #     ret, frame = cap.read()
    #     if not ret:
    #         continue

    #     final_frame, angle = Orchestrator(frame)

    #     datas.append(angle)
    #     deltaSec = time.time() - now

    #     time.sleep(0.05) #20fps

    return (len(datas) > 0, final_frame, sum(datas) / len(datas))



def prepare():

    global bridge
    global pub1
    global pub2
    global settings

    rospy.init_node('stella_line_tracking_node')
    bridge = CvBridge()
    pub1 = rospy.Publisher('stella_line_tracking', Image, queue_size=1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    settings = termios.tcgetattr(sys.stdin)

    pass


def publishTwist(linear = 0.0, angular = 0.0):

    global twist

    twist.linear.x = linear
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular

    pub2.publish(twist)


if __name__=="__main__":

    prepare()
    
    # testDrive()
    drive()
    # testCam()

    pass
