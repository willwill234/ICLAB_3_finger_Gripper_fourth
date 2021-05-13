#!/usr/bin/python3
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
import cv2
import ConfigParser as configparser
from control_node import HiwinRobotInterface
from control_node.msg import robot_info
from comm_stm32 import Gripper #import gripper class
from beginner_mission.msg import GGCNN_Grasp
from beginner_mission.msg import GGCNN_Grasp_array
from beginner_mission.msg import ROI
from beginner_mission.msg import ROI_array
from hand_eye.srv import eye2base, eye2baseRequest

#from beginner_mission.msg import BoundingBoxes
DEBUG = True  # Set True to show debug log, False to hide it.
ItemNo = 0
positon = [0.0,36.8,11.35,180,0,0]
Goal = [0.0,36.8,11.35,180,0,0]
Current_pos = [0.0,0.0,0.0,0.0,0.0,0.0]

target_base = []
target_base_true = []
Now_pose = []
error_x = -0.2
error_y = -0.3
pic_height_1 = 49.3
pic_height_2 = 18.3
pic_pos_x = 0.0
pic_pos_y = 36.8
pic_pos_z = pic_height_1
BASE_X = 0.0
BASE_Y = 0.0
cam_ready = 0
gripper_mode = ''
tmp_gripper_mode = ''
grap_angle = 0.0
tmp_angle = 0.0
tmp = 0
fin_task = 0
no_yolo_object = 0
grap_length = 0.0
# -----curve fitting variable-----
global image_x, arm_x, image_y, arm_y
image_x = [132,136,570,572,732,733,1166,1171]
arm_x=[-28.70,-28.4,-4.7,-4.4,4.2,4.5,27.8,28.4]
image_y= [205,531,285,446,284,445,199,525]
arm_y=[53.6,35.8,49.65,40.8,49.9,41,54.9,37.1]
 
global pic_M, pic_C, pic_OB, pic_OB_2
pic_M = [0.0, 0.0]
pic_C = [0.0, 0.0]
pic_OB = [0.0, 0.0]
pic_OB_2 = [0.0, 0.0]
# --------------------------------

my_data = GGCNN_Grasp()
yolo_data = ROI_array()
# Now_pose = robot_info()
def arm_state_listener():
    rospy.Subscriber("/object/Grasp_Detect", GGCNN_Grasp, get_str_mid)
    rospy.Subscriber("/object/ROI_array", ROI_array, get_yolo_img)

def get_yolo_img(data):
    global yolo_data
    yolo_data = data.ROI_list
    # print(yolo_data)


# -----for image pixel to position parameter-----+
global A
# A = np.mat([[-3.66634499e-02, -9.99298279e-01,  7.66430305e-03,  8.93675779138 + error_y],
#             [ 9.99326946e-01, -3.66715773e-02, -9.22540985e-04,  0.4787482122 + error_x],
#             [ 1.20295570e-03,  7.62532103e-03,  9.99970203e-01,  -11.0215100525],
#             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
A = np.mat([[-3.66634499e-02, -9.99298279e-01,  7.66430305e-03,  8.93675779138],
            [ 9.99326946e-01, -3.66715773e-02, -9.22540985e-04,  0.4787482122],
            [ 1.20295570e-03,  7.62532103e-03,  9.99970203e-01,  -11.0215100525],
            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

curr_path = os.path.dirname(os.path.abspath(__file__))
print(curr_path)
config = configparser.RawConfigParser()
# config.read(['img_trans.ini', curr_path])
config.read(curr_path + '\..\..\hand_eye\config\img_trans.ini')
# a00 = float(config.get("Section_A", "Key_1_1"))
# a01 = float(config.get("Section_A", "Key_1_2"))
# a02 = float(config.get("Section_A", "Key_1_3"))
# a10 = float(config.get("Section_A", "Key_2_1"))
# a11 = float(config.get("Section_A", "Key_2_2"))
# a12 = float(config.get("Section_A", "Key_2_3"))
# a20 = float(config.get("Section_A", "Key_3_1"))
# a21 = float(config.get("Section_A", "Key_3_2"))
# a22 = float(config.get("Section_A", "Key_3_3"))

a00 = float(config.get("External", "Key_1_1"))
a01 = float(config.get("External", "Key_1_2"))
a02 = float(config.get("External", "Key_1_3"))
a03 = float(config.get("External", "Key_1_4"))
a10 = float(config.get("External", "Key_2_1"))
a11 = float(config.get("External", "Key_2_2"))
a12 = float(config.get("External", "Key_2_3"))
a13 = float(config.get("External", "Key_2_4"))
a20 = float(config.get("External", "Key_3_1"))
a21 = float(config.get("External", "Key_3_2"))
a22 = float(config.get("External", "Key_3_3"))
a23 = float(config.get("External", "Key_3_4"))
A_mat = np.mat([a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23])
A_mat = A_mat.reshape(3,4)
print(A_mat)

# def cam2real(mx, my):
#     global pic_pos_x, pic_pos_y, pic_pos_z, A

#     c_x = 643.47548083
#     c_y = 363.67742746
#     f_x = 906.60886808
#     f_y = 909.34831447
#     k_1 = 0.16962942
#     k_2 = -0.5560001
#     p_1 = 0.00116353
#     p_2 = -0.00122694
#     k_3 = 0.52491878
    
#     camera_mat = np.array([[f_x, 0, c_x],
#                            [0, f_y, c_y],
#                            [0, 0, 1]])
#     dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
#     z_world = pic_pos_z

#     # print(mx,my)
#     img_pos = np.array([[[float(mx), float(my)]]])
#     img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

#     CAM_X = (img_pos[0] - c_x) * z_world / f_x
#     CAM_Y = (img_pos[1] - c_y) * z_world / f_y

#     A[0:3, 0:3] = A_mat
#     # print('A_MATRIX', A)
#     CAM_XY = np.mat([[CAM_X],[CAM_Y],[0],[1]])
#     END_XY = A * CAM_XY
#     END_X = END_XY[0, 0]
#     END_Y = END_XY[1, 0]
#     BASE_X = END_Y + pic_pos_x
#     BASE_Y = END_X + pic_pos_y
    
#     return BASE_X, BASE_Y

def curve():
    global all_ball, my_data, M, A, pic_pos_x, pic_pos_y, pic_pos_z, BASE_X, BASE_Y, cam_ready
 #=====================================================================
    # c_x = 643.47548083
    # c_y = 363.67742746
    # f_x = 906.60886808
    # f_y = 909.34831447
    # k_1 = 0.16962942
    # k_2 = -0.5560001
    # p_1 = 0.00116353
    # p_2 = -0.00122694
    # k_3 = 0.52491878
    c_x = 320.54516602
    c_y = 251.06906128
    f_x = 609.48925781
    f_y = 608.12738037
    k_1 = 0.16819103
    k_2 = -0.560136378
    p_1 = 0.000957010321
    p_2 = -0.000250189256
    k_3 = 0.519864386

    camera_mat = np.array([[f_x, 0, c_x],
                           [0, f_y, c_y],
                           [0, 0, 1]])
    dist_coef = np.array([k_1, k_2, p_1, p_2, k_3])
    # dist_coef = np.array([0, 0, 0, 0, 0])
    z_world = pic_pos_z
 #---------------------------------------------------------------------
    counter = 0
    dis_BaH = np.zeros(6) 
    # for item in my_data.GGCNN_Grasp:#--------------
    #     print(item)
    #     if float(item.probability) >= 0.7: #信心值大於預期目標 才做分析 取得該球中心座標
    #         exclusion_ball = 0
    #         # # -----取中心座標-----
    #         mx = float(item.xmax + item.xmin)/2
    #         my = float(item.ymax + item.ymin)/2

 #=====================================================================
    # print("XXXXXXXXXXXXXXXXXX", my_data.x)
    # print("YYYYYYYYYYYYYYYyyy", my_data.y)
    img_pos = np.array([[[my_data.x, my_data.y]]])
    img_pos = cv2.undistortPoints(img_pos, camera_mat, dist_coef, None, camera_mat)[0][0]

    CAM_X = (img_pos[0] - c_x) * z_world / f_x
    CAM_Y = (img_pos[1] - c_y) * z_world / f_y

    # CAM_X = (img_pos[0] - f_x) * z_world / c_x
    # CAM_Y = (img_pos[1] - f_y) * z_world / c_y

    # A[0:3, 0:3] = A_mat
    A[0:3, 0:4] = A_mat
    # print(A)
    CAM_XY = np.mat([[CAM_X],[CAM_Y],[0],[1]])
    END_XY = A * CAM_XY
    # END_XY = CAM_XY
    END_X = END_XY[0, 0]
    END_Y = END_XY[1, 0]
    BASE_X = END_Y + pic_pos_x
    BASE_Y = END_X + pic_pos_y
    # BASE_X = END_X + pic_pos_x
    # BASE_Y = END_Y + pic_pos_y

    cam_ready = 1
    # BASE_X = END_X 
    # BASE_Y = END_Y
    # BASE_X = END_Y 
    # BASE_Y = END_X
            # global H1,H2,H3,H4,H5,H6
            # global billiard_radius
            # H = [H1,H2,H3,H4,H5,H6]
            # for j in range(6):
            #     dis_BaH[j] = (((H[j][0] - BASE_X)**2 + (H[j][1] - BASE_Y)**2)**0.5)
            #     if dis_BaH[j] <= 1*billiard_radius :
            #         # print('i see a ball in hole~~~~~~~~~~~~~~~~~~~~~')
                
            #         exclusion_ball = 1
            # result = np.where(dis_BaH == np.amin(dis_BaH))
            # if exclusion_ball != 1:
            #     all_ball.append([str(item.Class), float(item.probability), BASE_X, BASE_Y, dis_BaH[result[0][0]]])
            #     counter += 1
    # print(' s', counter,'t')
    return BASE_X,BASE_Y,cam_ready


def get_str_mid(data):
    global my_data,target_base,cam_ready,grap_length,gripper_mode,tmp,grap_angle
    my_data = data
    camera_z = 43.7
    # camera_z = 48.3
    grap_angle = float(my_data.angle*90/1.57)
    # grap_length.append(my_data.length)
    # if len(grap_length) == 500:
    #     tmp = sum(grap_length)
    #     tmp = tmp / 500
    # print(tmp)
    # if tmp < 65:
    #     gripper_mode = '2_fingers'
    # else:
    #     gripper_mode = '3_fingers'
    grap_length = my_data.length
    if grap_length < 60:
        gripper_mode = '2_fingers'
    else:
        gripper_mode = '3_fingers'
            

    # print(gripper_mode)
    # print(grap_angle)
    baseRequest = eye2baseRequest()
    baseRequest.ini_pose = [my_data.x,my_data.y,camera_z] 
    target_base = pixel_z_to_base_client(baseRequest) #[x,y,z]

    cam_ready = cam_ready+1
    # return target_base, cam_ready, gripper_mode, grap_angle, grap_length

    #ball_quantite = curve()



def pixel_z_to_base_client(pixel_to_base):
    rospy.wait_for_service('robot/pix2base')
    # print("transsssssssss")
    try:
        pixel_z_to_base = rospy.ServiceProxy('robot/pix2base', eye2base)
        resp1 = pixel_z_to_base(pixel_to_base)
        return resp1.tar_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


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

def get_point():
    global ItemNo
    global BASE_X, BASE_Y
    # Arm_state = robot_ctr.get_robot_motion_state()
    # print("BASE_X:",BASE_X)
    # print("BASE_Y:",BASE_Y)
    time.sleep(2.0)

    # if Arm_state == 1:
    #     if ItemNo==0:
    #         positon =  [BASE_X, BASE_Y, 200, -180, 0, -180] #櫃子門把上方
    #         robot_ctr.Step_AbsPTPCmd(positon)
    #         ItemNo = 1
    #         print("task:0")

def gripper_test():
    global ItemNo
    # gripper.Send_Gripper_Command('drag_mode')
    # print("KKKKKKKKKKKKKKKKKKKK")
    # time.sleep(3)
    # # gripper.Send_Gripper_Command('Stop')
    # # print("JJJJJJJJJJJJJJJJJJJJJJ")
    # gripper.Send_Gripper_Command('2D_mode')
    # print("OOOOOOOOOOOOOOOOOOOOO")
    # time.sleep(3)
    gripper.Send_Gripper_Command('Stop')
    gripper.Send_Gripper_Command('3D_mode')
    time.sleep(2)
    gripper.Send_Gripper_Command('Stop')
    gripper.Send_Gripper_Command('2D_mode')
    time.sleep(3)
    gripper.Send_Gripper_Command('Stop')
    gripper.Send_Gripper_Command('2D_catch')
    time.sleep(0.2)
    gripper.Send_Gripper_Command('Stop')
    time.sleep(1)
    gripper.Send_Gripper_Command('Stop')
    gripper.Send_Gripper_Command('2D_loosen')
    time.sleep(2)
    gripper.Send_Gripper_Command('Stop')
    #============================================
    # gripper.Send_Gripper_Command('drag_mode')
    # gripper.Send_Gripper_Command('Stop')
    # time.sleep(2)
    # gripper.Send_Gripper_Command('2D_mode')
    # gripper.Send_Gripper_Command('Stop')
    # time.sleep(2)
    # gripper.Send_Gripper_Command('Catch_no3')
    # gripper.Send_Gripper_Command('Stop')
    # time.sleep(2)
    # gripper.Send_Gripper_Command('2D_catch')
    # gripper.Send_Gripper_Command('Stop')
    # time.sleep(1)
    # gripper.Send_Gripper_Command('2D_loosen')
    # gripper.Send_Gripper_Command('Stop')
    #========================================
    # gripper.Send_Gripper_Command('2D_mode')
    # gripper.Send_Gripper_Command('drag_mode')
    # time.sleep(2)
    # gripper.Send_Gripper_Command('2D_mode')
    # print("PPPPPPPPPPPPPPPPPPPPPP")

def currpoint():
    global ItemNo
    global BASE_X, BASE_Y, cam_ready, target_base_true,target_base,Now_pose
    Arm_state = robot_ctr.get_robot_motion_state()
    if Arm_state == 1:
        if ItemNo== 0:
            # print("BASE_X:",BASE_X)
            # print("BASE_Y:",BASE_Y)
            # positon =  [0, 36.8, 24.0, -180, 0, 90] #拍照位
            positon =  [-18.4445, 34.0519, 29.8943, -180, 0, 90] #拍照位
            robot_ctr.Step_AbsPTPCmd(positon)
            Now_pose = robot_ctr.Get_current_position()
            #if BASE_X and BASE_Y == 0.0:
            #    ItemNo = 0
            #else:
            #    ItemNo = 1
            # print(cam_ready)
            # print(Now_pose)
            for i in range(3):
                # print(int(Now_pose[i]))
                # print(int(positon[i]))
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 1
                else:
                    ItemNo = 0
            print("ItemNo" , ItemNo)

def point_test():
    global ItemNo, tmp_gripper_mode, yolo_data, no_yolo_object, fin_task
    global cam_ready, target_base_true, target_base, Now_pose, grap_angle, tmp_angle, gripper_mode,grap_length
    Arm_state = robot_ctr.get_robot_motion_state()
    if Arm_state == 1:
        if ItemNo== 0:
            positon =  [32.3744, 43.3260, 20.7778, -180, 0, -180]#拉抽屜點上方
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 1
                else:
                    ItemNo = 0
            print("task:0") 
        if ItemNo== 1:
            positon =  [32.3744, 43.3260, 10.6873, -180, 0, -180]#準備拉抽屜
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 2
                else:
                    ItemNo = 1
            print("task:1") 
        if ItemNo== 2:
            positon =  [8.4318, 43.3260, 10.6873, -180, 0, -180]#拉抽屜
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 3
                else:
                    ItemNo = 2
            print("task:2") 
        if ItemNo== 3:
            positon =  [8.4318, 43.3260, 25.7778, -180, 0, -180]#完成拉抽屜
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 4
                else:
                    ItemNo = 3
            print("task:3") 
        if ItemNo== 4:
            positon =  [0.000, 36.8000, 24.0500, -180, 0, 90]#Home
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    cam_ready = 0
                    ItemNo = 5
                else:
                    ItemNo = 4
            print("task:4") 
        if ItemNo== 5:
            positon =  [-18.4445, 34.0519, 29.8943, -180, 0, 90] #拍照位
            robot_ctr.Step_AbsPTPCmd(positon)
            # print(no_yolo_object)

            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    if len(yolo_data) == 0: 
                        no_yolo_object = no_yolo_object +1
                    if no_yolo_object >= 5000:
                        ItemNo = 12
                        no_yolo_object = 0
                    if cam_ready >= 150 and len(yolo_data) != 0:
                        tmp_gripper_mode = gripper_mode
                        tmp_angle = grap_angle 
                        target_base_true = target_base
                        no_yolo_object = 0
                        ItemNo = 6
                else:
                    ItemNo = 5
            print(tmp_angle)
            print(cam_ready)

            print("task:5")
        if ItemNo== 6:
            if tmp_gripper_mode == '3_fingers':
                gripper.Send_Gripper_Command('Stop')
                time.sleep(1)
                gripper.Send_Gripper_Command('3D_mode')
                time.sleep(2)
                gripper.Send_Gripper_Command('Stop')
                time.sleep(0.5)
                positon =  [target_base_true[0], target_base_true[1], 15, -180, 0, 90-tmp_angle]
            else:
                gripper.Send_Gripper_Command('Stop')
                time.sleep(1)
                gripper.Send_Gripper_Command('2D_mode')
                time.sleep(2)
                gripper.Send_Gripper_Command('Stop')
                time.sleep(0.5)
                gripper.Send_Gripper_Command('Catch_no3')
                time.sleep(4)
                gripper.Send_Gripper_Command('Stop')
                time.sleep(0.5)
                positon =  [target_base_true[0], target_base_true[1], 15, -180, 0, -tmp_angle]
            robot_ctr.Step_AbsPTPCmd(positon)
            # print("curr_pose",curr_pose)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 7
                else:
                    ItemNo = 6
            print("task:6") 

        if ItemNo== 7:
            if tmp_gripper_mode == '3_fingers':
                positon =  [target_base_true[0], target_base_true[1], 2.1923, -180, 0, 90-tmp_angle]
            else:
                positon =  [target_base_true[0], target_base_true[1], 1.3794, -180, 0, -tmp_angle]
            robot_ctr.Step_AbsPTPCmd(positon)
            # print("curr_pose",curr_pose)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 8
                else:
                    ItemNo = 7
            print("task:7") 
        
        if ItemNo== 8:
            if tmp_gripper_mode == '3_fingers':
                gripper.Send_Gripper_Command('Catch_all')
                time.sleep(10)
                gripper.Send_Gripper_Command('Stop')
                time.sleep(0.5)
            else:
                gripper.Send_Gripper_Command('2D_catch')
                time.sleep(10)
                gripper.Send_Gripper_Command('Stop')
                time.sleep(0.5)

            ItemNo = 9
            print("task:8")
        if ItemNo== 9:
            positon =  [-18.4445, 34.0519, 29.8943, -180, 0, 90-tmp_angle] #放置位
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 10
                else:
                    ItemNo = 9
            print("task:9")
        if ItemNo== 10:
            positon =  [30.1033, 42.0539, 25.9192, -180, 0, 90] 
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 11
                else:
                    ItemNo = 10
            print("task:10")

        if ItemNo== 11:
            if tmp_gripper_mode == '3_fingers':
                gripper.Send_Gripper_Command('Loosen_all')
            else:
                gripper.Send_Gripper_Command('2D_loosen')
                time.sleep(6)
                gripper.Send_Gripper_Command('Stop')
                time.sleep(1.5)
                gripper.Send_Gripper_Command('Loosen_no3')

            time.sleep(10)
            gripper.Send_Gripper_Command('Stop')
            time.sleep(0.5)
            gripper.Send_Gripper_Command('3D_mode')
            time.sleep(1)
            gripper.Send_Gripper_Command('Stop')
            time.sleep(1.5)
            gripper.Send_Gripper_Command('drag_mode')
            time.sleep(1.7)
            gripper.Send_Gripper_Command('Stop')
            time.sleep(0.5)
            ItemNo = 16
            print("task:11")
        if ItemNo== 12:
            positon = [18.4318, 43.3260, 25.9192, -180, 0, 0] #關抽屜上方
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 13
                else:
                    ItemNo = 12
            print("task:12")
        if ItemNo== 13:
            positon = [18.4318, 43.3260, 10.8035, -180, 0, 0] #準備關抽屜
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    ItemNo = 14
                else:
                    ItemNo = 13
            print("task:13")
        if ItemNo== 14:
            positon =  [46.5431, 43.3260, 10.8035, -180, 0, 0] #關抽屜
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    cam_ready = 0
                    ItemNo = 15
                else:
                    ItemNo = 14
            print("task:14")
        if ItemNo== 15:
            positon =  [25.1431, 43.3260, 25.8035, -180, 0, 0] #完成關抽屜
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    fin_task = 1
                    ItemNo = 16
                else:
                    ItemNo = 15
            print("task:15")
        if ItemNo== 16:
            positon =  [0.000, 36.8000, 24.0500, -180, 0, 90]#Home
            robot_ctr.Step_AbsPTPCmd(positon)
            for i in range(3):
                if int(Now_pose[i]) == int(positon[i])/10:
                    if fin_task == 1:
                        ItemNo = 17
                    else: 
                        cam_ready = 0
                        grap_length = 0.0
                        ItemNo = 5
                else:
                    ItemNo = 16
            print("task:16")
        return ItemNo

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
    # print(A_mat)
    gripper = Gripper()
    # gripper.Connect_to_Server()
    # gripper.Send_Gripper_Command('2D_mode')

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
    #test----gripper
    # for i in range(5):
    #     gripper.Send_Gripper_Command('Stop')
    #     time.sleep(1.2)
    #     gripper.Send_Gripper_Command('3D_mode')
    #     time.sleep(2)
    #     # gripper.Send_Gripper_Command('Stop')
    #     gripper.Send_Gripper_Command('drag_mode')
    #     time.sleep(3)
    #     gripper.Send_Gripper_Command('3D_mode')
    #     time.sleep(2.5)
    #     gripper.Send_Gripper_Command('Stop')
    #     time.sleep(2.5)
    #     gripper.Send_Gripper_Command('Catch_all')
    #     # gripper.Send_Gripper_Command('Catch_no1')
    #     # time.sleep(0.1)
    #     # gripper.Send_Gripper_Command('Catch_no2')
    #     # time.sleep(0.1)
    #     # gripper.Send_Gripper_Command('Catch_no3')
    #     time.sleep(10)
    #     gripper.Send_Gripper_Command('Loosen_all')
    #     time.sleep(10)
    #     # gripper.Send_Gripper_Command('Stop')
        # time.sleep(1)
    gripper.Send_Gripper_Command('Stop')
    time.sleep(1.2)
    gripper.Send_Gripper_Command('3D_mode')
    time.sleep(2)
    gripper.Send_Gripper_Command('Stop')
    time.sleep(1.7)
    gripper.Send_Gripper_Command('drag_mode')
    time.sleep(5)
    #=========================
    arm_state_listener()
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
            Now_pose = robot_ctr.Get_current_position()
            # currpoint()
            # gripper_test()
            point_test()
            # get_point()
            #test_task()
            #print("boxes:",boxes)
            #robot_ctr.Set_digital_input(2,1)
            # robot_outputs_state = robot_ctr.Get_current_robot_outputs()
            # robot_inputs_state = robot_ctr.Get_current_robot_inputs()
            # digital_output_state = robot_ctr.Get_current_digital_outputs()
            #print("robot outputs state:",robot_outputs_state)
            #print("robot inputs state:",robot_inputs_state)

            #positon = [0.0,0.0,10.0,-180,0,0]
            #robot_ctr.Step_AbsPTPCmd(positon)

            # pose = robot_ctr.Get_current_position()
            # print("pose:",pose)

    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass
    rospy.spin()

