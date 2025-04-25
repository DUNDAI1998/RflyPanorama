'''
use Rflysim APIs to get imgs, positions, velocity, etc.
Refs: https://rflysim.com/doc/zh/RflySimAPIs/8.RflySimVision/Index.pdf; https://rflysim.com/doc/zh/RflySimAPIs/8.RflySimVision/0.ApiExps/1-UsageAPI/Readme.pdf
imgs: img_{index of cameras}_{timestamp}.jpg
label format: timestamp, x, y, z, vx, vy, vz, roll(rad), pitch(rad), yaw(rad), wx, wy, wz (data of PX4)
cam format: [mapping_coeffients_i(4x1), ImageSize_i(2x1), Distortion_Center_i(2x1), StretchMatrix_i(4x1), roll_i(reg), pitch_i(reg), yaw_i(reg), tx_i(m), ty_i(m), tz_i(m) for i in range(4)]
            -> 4 cams in total ; roll pitch yaw and t are relative to the body frame of the drone
'''

import PX4MavCtrlV4 as PX4MavCtrl
import VisionCaptureApi
import UE4CtrlAPI

import sys
import os
import time
import cv2
# import multiprocessing
import json
# from msvcrt import getch
import subprocess

# sensor
vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad(jsonPath='./config/Config.json') # create sensor
ue = UE4CtrlAPI.UE4CtrlAPI()

def change_scene(ind_scene):
    current_path = os.path.abspath(os.path.dirname(__file__))
    path = current_path + f'\config\scene{ind_scene:03}.bat'
    subprocess.run(['cmd.exe', '/c', path])
################# params setting #################################
ind_scene = 2  # scene index
ind_traj = 8   # trajectory index
change_scene(ind_scene)
time.sleep(100)  # wait for UE4 to load the scene

num_cam = 4

path = f'./dataset/scene{ind_scene:03}/traj{ind_traj:03}/'
if not os.path.exists(path):
    os.makedirs(path)

# save camera intrinsics
cam_infos_path = f'{path}cam_infos.txt'
lines_cam_infos = []
with open('./config/Config.json', 'r') as f:
    cam_data = json.load(f)
for i in range(num_cam):
    '''
    calibrated by checkboard in RflySim3D. suited for fisheye cams with fov of 200°, width of 640 and height of 640.
    '''
    mapping_coeffients = [167.6389, -0.0020, 4.3925e-7, -6.7036e-9]
    ImageSize = [640, 640]
    Distortion_Center = [320.7997, 321.6138]
    StretchMatrix = [1.0, 0.0, 0.0, 1.0]  # resized from 2x2 matrix

    roll, pitch, yaw = cam_data['VisionSensors'][i]['SensorAngEular']
    tx, ty, tz = cam_data['VisionSensors'][i]['SensorPosXYZ']
    lines_cam_infos.append(''.join(
        f"{mapping_coeffients[0]:.4f} {mapping_coeffients[1]:.4f} {mapping_coeffients[2]:.4f} {mapping_coeffients[3]:.4f} {ImageSize[0]:.4f} {ImageSize[1]:.4f} {Distortion_Center[0]:.4f} {Distortion_Center[1]:.4f} {StretchMatrix[0]:.4f} {StretchMatrix[1]:.4f} {StretchMatrix[2]:.4f} {StretchMatrix[3]:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f} {tx:.4f} {ty:.4f} {tz:.4f}\n"))
with open(cam_infos_path, 'w') as f:
    f.writelines(lines_cam_infos)

f = float(cam_data['VisionSensors'][0]['DataCheckFreq'])   # Hz
interval_time = 1.0/f

# Init MAVLink data receiving loop
mav = PX4MavCtrl.PX4MavCtrler(1)
mav.InitMavLoop()
time.sleep(3)
mav.initOffboard() # offboard
mav.SendMavArm(True) # disarm


# mav.SendPosNED(0,0,-25,0)
# time.sleep(10)  # for the time to execute SendPosNED commands
isSuss = vis.sendReqToUE4()  # send request to Rflysim3D
if not isSuss:  
    print('The request for image capture failed.')
    sys.exit(0)
vis.startImgCap()  # after taking off


def record_data(num_cam, interval_time, path):
    print('Start recording data...')
    while True:
        start_time = time.time()

        img_list = vis.Img
        img_flag_list = vis.hasData
        # lat, lon, alt, _, _, velx, vely, velz, _ = mav.uavPosGPS # check the source code: lat(reg), lon(reg), alt, time_boot, relative_alt, vx, vy, vz, hdg 
        posx, posy, posz = mav.uavPosNED
        velx, vely, velz = mav.uavVelNED
        roll, pitch, yaw = mav.uavAngEular
        omegax, omegay, omegaz = mav.uavAngRate
        lines_label_px4 = ''.join(f"{start_time:.6f} {posx:.4f} {posy:.4f} {posz:.4f} {velx:.4f} {vely:.4f} {velz:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f} {omegax:.4f} {omegay:.4f} {omegaz:.4f}\n")
        
        with open(path + f'label_{start_time:.6f}.txt', 'w') as f:
            f.writelines(lines_label_px4)

        for i in range(num_cam):
            if img_flag_list[i]:
                img = img_list[i]
                cv2.imwrite(path + f'img_{i}_{start_time:.6f}.jpg', img)  


        end_time = time.time()
        while end_time - start_time < interval_time:  # f Hz 
            end_time = time.time()

# def control_traj():
#     print('Start controlling the drone...')
#     while True:
#         ch = getch()
#         if ch == b'w':
#             print('forward')
#             mav.SendVelNEDNoYaw(vx=8, vy=0, vz=0)  # forward
#         elif ch == b's':
#             print('backward')
#             mav.SendVelNEDNoYaw(vx=-8, vy=0, vz=0)  # backward
#         elif ch == b'a':
#             print('left')
#             mav.SendVelNEDNoYaw(vx=0, vy=-8, vz=0)  # left
#         elif ch == b'd':
#             print('right')
#             mav.SendVelNEDNoYaw(vx=0, vy=8, vz=0)  # right
#         elif ch == b' ':
#             print('up')
#             mav.SendVelNEDNoYaw(vx=0, vy=0, vz=-8)  # up
#         elif ch == b'z':
#             print('down')
#             mav.SendVelNEDNoYaw(vx=0, vy=0, vz=8)  # down
#         elif ch == b'q':
#             print('right yaw')
#             mav.SendAttPX4(att=[0,0,0.3], CtrlFlag=3)  # right yaw
#         elif ch == b'e':
#             print('left yaw')
#             mav.SendAttPX4(att=[0,0,-0.3], CtrlFlag=3)  # left yaw
#         elif ch == b'b':
#             print('be still')
#             mav.SendVelNEDNoYaw(vx=0, vy=0, vz=0)  # be still
#             mav.SendAttPX4(att=[0,0,0], CtrlFlag=3)  # be still
#         elif ch == b'r':
#             print('stop moving')
#             break



if __name__ == '__main__':
    ################### get dataset ####################
    # parallel processing
    
    # record_process = multiprocessing.Process(target=record_data, args=(num_cam, interval_time, path))
    # control_process = multiprocessing.Process(target=control_traj, args=())

    # record_process.start()
    # control_process.start()
    # record_process.join()
    # control_process.join()

    # the control of trajectory can be done by QGC
    record_data(num_cam, interval_time, path)
