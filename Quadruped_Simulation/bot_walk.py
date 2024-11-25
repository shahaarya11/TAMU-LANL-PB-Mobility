import time
import numpy as np
import pybullet as p
import pybullet_data
import csv
import os
import math

from src.kinematic_model import robotKinematics
from src.pybullet_debugger import pybulletDebug  
from src.gaitPlanner import trotGait
from pydualsense import pydualsense, TriggerModes
ds = pydualsense() # open controller
ds.init() # initialize controller
def rendering(render):
    """Enable/disable rendering"""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, render)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, render)

def robot_init( dt, body_pos, fixed = False ):
    physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
    # turn off rendering while loading the models
    rendering(0)

    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(0)
    p.setPhysicsEngineParameter(
        fixedTimeStep=dt,
        numSolverIterations=100,
        enableFileCaching=0,
        numSubSteps=1,
        solverResidualThreshold=1e-10,
        erp=1e-1,
        contactERP=0.0,
        frictionERP=0.0,
    )
    # add floor
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.loadURDF("plane.urdf")
    startPos = [0,0,0.25]
    # add robot
    
    body_id = p.loadURDF(r"C:\Users\12105\OneDrive\Documents\MEEN 401\hello\DingoQuadruped\dingo_ws\src\dingo_description\urdf\dingo.urdf",body_pos, useFixedBase=fixed)
    #body_id = p.loadURDF(r"C:\Users\12105\OneDrive\Documents\MEEN 401\hello\Stride_bot\stridebot.urdf", body_pos, useFixedBase=fixed)
    joint_ids = []
    
    #robot properties
    maxVel = 3.703 #rad/s
    for j in range(p.getNumJoints(body_id)):
        p.changeDynamics( body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics( body_id, j, maxJointVelocity=maxVel)
        joint_ids.append( p.getJointInfo(body_id, j))
    rendering(1)
    return body_id, joint_ids

def robot_stepsim( body_id, body_pos, body_orn, body2feet ):
    #robot properties
    fr_index, fl_index, br_index, bl_index = 3, 7, 11, 15
    maxForce = 2 #N/m
    
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    fr_angles, fl_angles, br_angles, bl_angles , body2feet_ = robotKinematics.solve( body_orn , body_pos , body2feet )
    #move movable joints
    for i in range(3):
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, targetPosition = fr_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, targetPosition = fl_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, targetPosition = br_angles[i] , force = maxForce) 
        p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, targetPosition = bl_angles[i] , force = maxForce)

    p.stepSimulation()
    
    return body2feet_

def robot_quit():
    p.disconnect()
        
        
if __name__ == '__main__':
    dT = 0.005
    bodyId, jointIds = robot_init( dt = dT, body_pos = [0,0,0.18], fixed = False )
    pybulletDebug = pybulletDebug()
    robotKinematics = robotKinematics()
    trot = trotGait()

    """initial foot position"""
    #foot separation (Ydist = 0.16 -> theta =0) and distance to floor
    Xdist, Ydist, height = 0.24, 0.13, 0.15
    #body frame to foot frame vector
    bodytoFeet0 = np.matrix([[ Xdist/2. , -Ydist/2. , height],
                            [ Xdist/2. ,  Ydist/2. , height],
                            [-Xdist/2. , -Ydist/2. , height],
                            [-Xdist/2. ,  Ydist/2. , height]])

    offset = np.array([0.5, 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
    footFR_index, footFL_index, footBR_index, footBL_index = 3, 7, 11, 15
    T = 0.5 #period of time (in seconds) of every step
    
    N_steps=500000
    from time import sleep

    from dualsense_controller import DualSenseController

    # list availabe devices and throw exception when tzhere is no device detected
    device_infos = DualSenseController.enumerate_devices()
    if len(device_infos) < 1:
        raise Exception('No DualSense Controller available.')

    # flag, which keeps program alive
    #is_running = True

    # create an instance, use fiŕst available device, in this case we're using usb connected device
    controller = DualSenseController()
    controller.activate()

    # this is bad practice, but we need these variables to initialize and maintain value throughout
    # the run simulation so making global variables allows us to initialize and then continuously update
    # variables throughout the sim
    left_stick_y_global = 0
    left_stick_x_global = 0
    # callback when legt joystick is moved in y direction
    def on_left_stick_y_changed(left_stick_y):
            global left_stick_y_global
            left_stick_y_global = left_stick_y
            print(left_stick_y_global)
    # callback when legt joystick is moved in x direction
    def on_left_stick_x_changed(left_stick_x):
            global left_stick_x_global
            left_stick_x_global = left_stick_x
            print(left_stick_x_global)
        
    
    # controller.left_trigger.on_change(on_left_trigger)

    # calls function from the dualsense_controller library that reads controller input
    # when function senses specified controller input, it calls the callback function that updates the global variables
    controller.left_stick_x.on_change(on_left_stick_x_changed)
    controller.left_stick_y.on_change(on_left_stick_y_changed)
    #print(left_stick_y_global)
    #print(left_stick_x_global)
    for k_ in range(0,N_steps):
        # takes input from the user
        pos , orn , L , angle , Lrot , T , sda, offset= pybulletDebug.cam_and_robotstates(bodyId) 
        #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        bodytoFeet = trot.loop( left_stick_y_global , angle , left_stick_x_global , T , offset , bodytoFeet0 , sda) 
        #print(left_stick_y_global)
        robot_stepsim( bodyId, pos, orn, bodytoFeet ) #simulates the robot to the target position
    controller.deactivate()
    robot_quit()
