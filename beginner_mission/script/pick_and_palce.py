# coding=utf-8
import rospy
import sys
import time
import threading
import argparse
import numpy as np
import math
import enum
import time
import os
from control_node import HiwinRobotInterface
from comm_stm32 import Gripper #import gripper class
DEBUG = True  # Set True to show debug log, False to hide it.
ItemNo = 0
positon = [0.0,36.8,11.35,180,0,0]
Goal = [0.0,36.8,11.35,180,0,0]
Current_pos = [0.0,0.0,0.0,0.0,0.0,0.0]

##-----------switch define------------##
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

def test_task():
    global ItemNo
    Arm_state = robot_ctr.get_robot_motion_state()
    if Arm_state == 1:
        if ItemNo==0:
            positon =  [158, 459.8, 200, -180, 0, -180] #櫃子門把上方
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 1
            print("task:0")
        elif ItemNo==1:
            positon =  [158, 459.8, 132.5, -180, 0, -180] #夾具門把位
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 2
            print("task:1")
        elif ItemNo==2:
            positon =  [-82, 459.8, 132.5, -180, 0, -180]  #拉開櫃子
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 3
            print("task:2")
        elif ItemNo==3:
            positon =  [-68, 459.8, 132.5, -180, 0, -180] # 偏差回位
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 4
            print("task:3")
        elif ItemNo==4:
            positon =  [-68, 459.8, 248.1, -180, 0, -180] # 上升
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 5
            print("task:4")
        elif ItemNo==5:
            positon =  [0, 368, 293, 180, 0, 90] #back home
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 6
            print("task:5")
        elif ItemNo==6:
            positon =  [56.2, 459.8, 200, -180, 0, 0]  #關上櫃子起點(上方)
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 7 
            print("task:6")
        elif ItemNo==7:
            positon =  [56.2, 459.8, 144.4, -180, 0, 0] #關上櫃子起點
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 8
            print("task:7")
        if ItemNo==8:
            positon =  [298.4, 459.8, 144.4, -180,  0, 0] #推進
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 9
            print("task:8")
        if ItemNo==9:
            positon =  [120, 459.8, 144.4, -180,  0, 0] #關完櫃子
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 10
            print("task:9")
        if ItemNo==10:
            positon =  [0, 368, 293, 180, 0, 90] #back home
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 11
            print("task:10")                  
        elif ItemNo==11:
            robot_ctr.Set_operation_mode(0)
            robot_ctr.Go_home()
            ItemNo = 0
            print("task:11")

if __name__ == '__main__':
    gripper = Gripper()
    gripper.Send_Gripper_Command('2D_mode')
    arg_parser = argparse.ArgumentParser("Driver Node")
    arg_parser.add_argument("--robot_ip", help="IP addr of the robot",
                            type=str)
    arg_parser.add_argument("--robot_name", help="Name of the robot", type=str)
    arg_parser.add_argument("--control_mode", help="Default is 1, set it to 0 if you do not want to control the robot, but only to monitor its state.",
                            type=bool, default=1, required=False)
    arg_parser.add_argument("--log_level", help="Logging level: INFO, DEBUG",
                            type=str, default="INFO", required=False)
    arg_parser.add_argument("__name")
    arg_parser.add_argument("__log")
    args = arg_parser.parse_args()

    # Extract the necessary arguments
    robot_ip = args.robot_ip
    robot_name = args.robot_name
    control_mode = int(args.control_mode)
    if args.log_level == "DEBUG":
        log_level = rospy.DEBUG
    elif args.log_level == "ERROR":
        log_level = rospy.ERROR
    else:
        log_level = rospy.INFO
    
    # Start the ROS node
    rospy.init_node('hiwin_robot_sdk_'+robot_name,
                    log_level=log_level,
                    disable_signals=True)
    if rospy.get_param("use_sim_time", False):
        rospy.logwarn("use_sim_time is set!!!")

    robot_ctr = HiwinRobotInterface(robot_ip=robot_ip, connection_level=control_mode,name=robot_name)
    robot_ctr.connect()
    try:
        if robot_ctr.is_connected():
            robot_ctr.Set_operation_mode(0)
            # set tool & base coor
            #tool_coor = [0,0,180,180,0,0]
            #base_coor = [0,0,0,0,0,0]
            robot_ctr.Set_base_number(0)
            #base_result = robot_ctr.Define_base(1,base_coor)
            robot_ctr.Set_tool_number(5)
            #tool_result = robot_ctr.Define_tool(1,tool_coor)

            robot_ctr.Set_operation_mode(1)
            robot_ctr.Set_override_ratio(30)  #手臂程式速度
            robot_ctr.Set_acc_dec_ratio(50) #手臂加速度

            # robot_ctr.Set_robot_output(2,False)
            # robot_ctr.Set_robot_output(3,False)
            # robot_ctr.Set_robot_output(4,False)
            robot_ctr.Set_digital_output(1,False)
            robot_ctr.Set_digital_output(2,False)
            robot_ctr.Set_digital_output(3,False)
        while(1):
            test_task()
            #print("boxes:",boxes)
            #robot_ctr.Set_digital_input(2,1)
            # robot_outputs_state = robot_ctr.Get_current_robot_outputs()
            # robot_inputs_state = robot_ctr.Get_current_robot_inputs()
            # digital_output_state = robot_ctr.Get_current_digital_outputs()
            #print("robot outputs state:",robot_outputs_state)
            #print("robot inputs state:",robot_inputs_state)

            #positon = [0.0,0.0,10.0,-180,0,0]
            #robot_ctr.Step_AbsPTPCmd(positon)

            pose = robot_ctr.Get_current_position()
            print("pose:",pose)
        rospy.spin()
    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass

