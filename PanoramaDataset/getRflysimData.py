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
import numpy as np
import json
import math
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
ind_traj =  11  # trajectory index
num_cam = 4

change_scene(ind_scene)
time.sleep(100)  # wait for UE4 to load the scene



path = f'./fisheye_dataset/scene{ind_scene:03}/traj{ind_traj:03}/'
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


isSuss = vis.sendReqToUE4()  # send request to Rflysim3D
if not isSuss:  
    print('The request for image capture failed.')
    sys.exit(0)
vis.startImgCap()  # after taking off


def record_data(num_cam, interval_time, path):
    print('Start recording data...')
    while True:
        startTime = time.time()

        img_list = vis.Img
        img_flag_list = vis.hasData
        # lat, lon, alt, _, _, velx, vely, velz, _ = mav.uavPosGPS # check the source code: lat(reg), lon(reg), alt, time_boot, relative_alt, vx, vy, vz, hdg 
        posx, posy, posz = mav.uavPosNED
        velx, vely, velz = mav.uavVelNED
        roll, pitch, yaw = mav.uavAngEular
        omegax, omegay, omegaz = mav.uavAngRate
        lines_label_px4 = ''.join(f"{startTime:.6f} {posx:.4f} {posy:.4f} {posz:.4f} {velx:.4f} {vely:.4f} {velz:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f} {omegax:.4f} {omegay:.4f} {omegaz:.4f}\n")

        with open(path + f'label_{startTime:.6f}.txt', 'w') as f:
            f.writelines(lines_label_px4)

        for i in range(num_cam):
            if img_flag_list[i]:
                img = img_list[i]
                cv2.imwrite(path + f'img_{i}_{startTime:.6f}.jpg', img)  


        end_time = time.time()
        while end_time - startTime < interval_time:  # f Hz 
            end_time = time.time()


def sendCarPosAng(ue,copterID,VehicleType,MotorRPMSMean,PosE,angEuler):
    '''
    send position and attitude of 8 cars
    '''
    row,col=PosE.shape
    if col !=3:
        return False
    for i in range(row):
        ue.sendUE4Pos2Ground(copterID[i].astype(int),VehicleType[i].astype(int),MotorRPMSMean[i].astype(int),PosE[i,:].tolist(),angEuler[i].tolist())

def ctrl_moving_car(num_car=8):
    '''
    The initial position of 8 cars is set in scene002, i.e. OldFactory.
    '''
    copterID = np.zeros(num_car + 1)
    for i in range(num_car):
        copterID[i] = i + 1
    VehicleType = np.array([310, 300051, 200051, 51, 51, 100051, 100051, 51, 100051])
    MotorRPMSMean = np.zeros(num_car)
    PosE = np.zeros((num_car, 3))
    angEuler = np.zeros((num_car, 3))

    time_interval = 0.02

    startTime = time.time()
    lastTime = time.time()
    while True:
        lastTime = lastTime + time_interval
        sleep_time = lastTime - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            lastTime = time.time()

        current_time = time.time() - startTime  # simulation time, same as clock in Simulink

        # position of 8 cars   
        Car1Const = np.array([60, -117, -1])     # initial position of car #1
        slopeInput = np.array([current_time*1/15.0, 0, 0])
        Car1SinInput = np.array([20*math.sin(0.2*current_time - math.pi/2) + 0, 0, 0])
        Car1PosE = Car1Const + Car1SinInput + slopeInput
        PosE[0, :] = Car1PosE
        
        Car2Const = np.array([30, -117, -1])
        Car2PosE = Car2Const + slopeInput  
        PosE[1, :] = Car2PosE
        
        Car3Const = np.array([15.7,-119,-1])
        Car3SinInput = np.array([5*math.sin(0.3*current_time + math.pi/3) + 0, 0, 0])
        Car3PosE = Car3Const + slopeInput + Car3SinInput
        PosE[2, :] = Car3PosE
        angEuler[2, :] = np.array([0, 0, 0])
        
        Car4Const = np.array([17.7, -115, -1]) 
        Car4SinInput = np.array([5*math.sin(0.3*current_time + 0) + 0, 0, 0])
        Car4PosE = Car4Const + slopeInput + Car4SinInput
        PosE[3, :] = Car4PosE    
        angEuler[3, :] = np.array([0, 0, 0])
        
        Car5Const = np.array([2, -119, -1])
        Car5SinInput = Car3SinInput 
        Car5PosE = Car5Const + slopeInput + Car5SinInput
        PosE[4, :] = Car5PosE
        angEuler[4, :] = np.array([0, 0, 0])   
        
        Car6Const = np.array([4, -115, -1]) 
        Car6SinInput = Car4SinInput
        Car6PosE = Car6Const + slopeInput + Car6SinInput
        PosE[5, :] = Car6PosE
        angEuler[5, :] = np.array([0, 0, 0])     
        
        Car7Const = np.array([-12, -117, -1]) 
        Car7SinInput = np.array([5*math.sin(0.3*current_time + math.pi/6) + 0, 0, 0])
        Car7PosE = Car7Const + slopeInput + Car7SinInput
        PosE[6, :] = Car7PosE
        angEuler[6, :] = np.array([0, 0, 0])       
        
        Car8Const = np.array([-24, -117, -1])
        Car8SinInput = np.array([5*math.sin(0.3*current_time + math.pi/3.5) + 0, 0, 0])
        Car8PosE = Car8Const + slopeInput + Car8SinInput
        PosE[7, :] = Car8PosE
        angEuler[7, :] = np.array([0, 0, 0]) 

        sendCarPosAng(ue, copterID[1:], VehicleType, MotorRPMSMean, PosE, angEuler)  


if __name__ == '__main__':
    ################### collect dataset ####################
    if ind_scene == 2:
        print('Start moving cars and recording data...')
        import threading

        thread_moving = threading.Thread(target=ctrl_moving_car)
        thread_recording = threading.Thread(target=record_data, args=(num_cam, interval_time, path))

        thread_moving.start()
        thread_recording.start()

        thread_moving.join()
        thread_recording.join() 
    else:
        print("only record data....")
        record_data(num_cam, interval_time, path)
