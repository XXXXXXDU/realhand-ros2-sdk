#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Header
from sensor_msgs.msg import JointState
import std_msgs.msg

import time

import threading

import signal
import sys
from builtin_interfaces.msg import Time

show_count=0
show_count_obj=0
show_step=0
joint_state = JointState() 
hand = {
    "joint1": 255,   # thumb base flexion (bend)
    "joint2": 128,   # thumb lateral splay (ab/adduction)
    "joint3": 255,   # index finger base flexion
    "joint4": 255,   # middle finger base flexion
    "joint5": 255,   # ring finger base flexion
    "joint6": 255,   # little finger base flexion
    "joint7": 128,   # index finger splay (ab/adduction)
    "joint8": 128,   # middle finger splay (ab/adduction)
    "joint9": 128,   # ring finger splay (ab/adduction)
    "joint10": 255,  # thumb rotation
}

def main(args=None):

    rclpy.init(args=args)
    node = Node("dong_test_sender")
    rate = 1.0 / 30  # 60 FPS
    #pub = node.create_publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)
    pub = node.create_publisher(JointState, '/cb_right_hand_control_cmd', 10)
    now = node.get_clock().now()
    
    joint_state.header = Header()

    joint_state.header.stamp = Time(sec=int(now.nanoseconds // 1e9), 
                            nanosec=int(now.nanoseconds % 1e9))
    joint_state.name=list(hand.keys())
    joint_state.velocity = [0] * len(joint_state.position)  # Same length as position; fill all with 0
    joint_state.effort   = [0] * len(joint_state.position)  # Set zero effort for each joint
    pub.publish(joint_state)
    count = 0
    while rclpy.ok():
        position =show_left()
        if(position is not None):
            joint_state.position = position
        pub.publish(joint_state)
        time.sleep(rate)
        count = count + 1
        print(count)

def show_left():
    global show_count
    global show_count_obj
    global show_step
    global hand
    print(show_step)
    show_count= show_count+1
    if(show_count>=show_count_obj):
        show_count=0
        if(show_step==0):
            show_step=show_step+1
            show_count_obj = 100
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==1): # // Curl little & ring fingers.
            show_step=show_step+1
            show_count_obj = 10
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint5'] = 0
            hand['joint6'] = 0
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==2): #// Place the thumb over the little & ring fingers.
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint10'] = 80
            return list(hand.values())
        elif(show_step==3): #// Tilt the index & middle fingers to one side.
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 200
            return list(hand.values())
        elif(show_step==4): #// Tilt to the other side.
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 50
            return list(hand.values())
        elif(show_step==5): #// Bring both back to center.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 128
            return list(hand.values())
        elif(show_step==6): #// Form a Y/V shape with the index & middle fingers.
            show_step=show_step+1
            show_count_obj = 2  
            hand['joint7'] = 50
            return list(hand.values())
        elif(show_step==7): #// Close the Y shape.
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 128
            return list(hand.values())
        elif(show_step==8): #// Form the Y/V shape again.
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 50
            return list(hand.values())
        elif(show_step==9): #// Close the Y shape.
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 128
            return list(hand.values())
        elif(show_step==10): #// Alternate bending/straightening of middle & index (round 1).
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==11): #// Alternate bending/straightening of middle & index (round 2).
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==12): #// Alternate bending/straightening of middle & index (round 3).
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==13): #// Alternate bending/straightening of middle & index (round 4).
            show_step=show_step+1
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==14): #// Curl the thumb.
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint10'] = 80
            return list(hand.values())
        elif(show_step==15): #// Tuck the thumb into the palm.
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==16): #// Curl all four fingers.
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 10
            hand['joint4'] = 10
            hand['joint5'] = 10
            hand['joint6'] = 10
            return list(hand.values())
        elif(show_step==17): #// Sequentially open the four fingers and the thumb.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==18): #// 1
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==19): #// 2
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==20): #// 3
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            return list(hand.values())
        elif(show_step==21): #// 40
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 110
            hand['joint10'] = 240
            return list(hand.values())
        elif(show_step==22): #// Bring the thumb together (adduct).
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 10
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==23): #// Rotate the thumb to face the palm.
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            hand['joint2'] = 10
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==24): #// Return to the initial position in two steps (step 1).
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 0
            hand['joint2'] = 240
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==25): #// 1
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==26): #// 2
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 200
            hand['joint8'] = 200
            hand['joint9'] = 200
            return list(hand.values())
        elif(show_step==27): #// 3
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 80
            hand['joint8'] = 80
            hand['joint9'] = 80
            return list(hand.values())
        elif(show_step==28): #// 4
            show_step=show_step+1
            show_count_obj = 20
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            return list(hand.values())
        elif(show_step==29): #// Sequentially curl the four fingers (prep).
            show_step=show_step+1
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==30): #// Curl the four fingers.
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==31): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==32): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==33): #// Sequentially curl the four fingers.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 0
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==34): #// Sequentially curl the four fingers.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 0
            return list(hand.values())
        elif(show_step==35): #// Sequentially curl the four fingers.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 0
            return list(hand.values())
        elif(show_step==36): #// Sequentially curl the four fingers.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 0
            return list(hand.values())
        elif(show_step==37): #// Curl the thumb.
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            return list(hand.values())
        elif(show_step==38): #// Open the index & little fingers.
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 250
            hand['joint2'] = 230
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==39): #// Open the index & little fingers.
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 250
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==40): #// Place the thumb on top (gesture "666").
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 10
            hand['joint2'] = 40
            hand['joint10'] = 60
            return list(hand.values())
        elif(show_step==41): #// Move fingers left/right.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 80
            hand['joint9'] = 200
            return list(hand.values())
        elif(show_step==42): #// Move fingers left/right.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 200
            hand['joint9'] = 80
            return list(hand.values())
        elif(show_step==43): #// Move fingers left/right.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 80
            hand['joint9'] = 200
            return list(hand.values())
        elif(show_step==44): #// Move fingers left/right.
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 200
            hand['joint9'] = 80
            return list(hand.values())
        elif(show_step==45): #// Move fingers left/right (return to center).
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 128
            hand['joint9'] = 128
            return list(hand.values())
        elif(show_step==46): #// Open/spread.
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==47): #// Pinch thumb & index finger.
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 130
            hand['joint2'] = 130
            hand['joint3'] = 130
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint10'] = 90
            return list(hand.values())
        elif(show_step==48): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 120
            return list(hand.values())
        elif(show_step==49): #// Pinch thumb & middle finger.
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 120
            hand['joint4'] = 130
            hand['joint10'] = 60
            return list(hand.values())
        elif(show_step==50): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 145
            return list(hand.values())
        elif(show_step==51): #// Pinch thumb & ring finger.
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 113
            hand['joint2'] = 103
            hand['joint5'] = 128
            hand['joint10'] = 42
            return list(hand.values())
        elif(show_step==52): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==53): #// Pinch thumb & little finger.
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 118
            hand['joint2'] = 103
            hand['joint6'] = 120
            hand['joint10'] = 22
            return list(hand.values())
        elif(show_step==54): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        # elif(show_step==55): #// Pinch thumb & little finger.
        #     show_step=show_step+1
        #     show_count_obj = 40
        #     return[160,250,250,250,160, 60,128,128,128,128,50, 0, 0, 0, 0,100,250,250, 250, 80]
        # elif(show_step==56): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 20
        #     return[250,250,250,250,250,130,128,128,128,128,100, 0, 0, 0, 0,250,250,250,  50, 250]
        # elif(show_step==57): #// Pinch thumb & ring finger.
        #     show_step=show_step+1
        #     show_count_obj = 35
        #     return[160,250,250,150,250,100,128,128,128,128,50, 0, 0, 0, 0,100,250,250, 80, 250]
        # elif(show_step==58): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 20
        #     return[250,250,250,250,250,180,128,128,128,128,100, 0, 0, 0, 0, 250,250, 50,  250, 250]
        # elif(show_step==59): #// Pinch thumb & middle finger.
        #     show_step=show_step+1
        #     show_count_obj = 35
        #     return[160,250,150,250,250,135,128,128,128,128,70, 0, 0, 0, 0,100,250,85, 250, 250]
        # elif(show_step==60): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 20
        #     return[250,250,250,250,250,220,128,128,128,128,100, 0, 0, 0, 0, 250, 50,250,  250, 250]
        # elif(show_step==61): #// Pinch thumb & index finger.
        #     show_step=show_step+1
        #     show_count_obj = 35
        #     return[165,150,250,250,250,170,128,128,128,128,70, 0, 0, 0, 0,100,80,250, 250, 250]
        # elif(show_step==62): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 60
        #     return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250,250,  250, 250]
        # else:
        #     show_step=0

def signal_handler(sig, frame):

    print('You pressed Ctrl+C!')

    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    try:
        print("测试中")
        main()
    except KeyboardInterrupt:
         print("Caught KeyboardInterrupt, exiting gracefully.")
