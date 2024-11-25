import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0.25]
startOrientation = p.getQuaternionFromEuler([0,0,0])
fixedBase = True
boxId = p.loadURDF(r"C:\Users\12105\OneDrive\Documents\MEEN 401\URDF test\robot_arm_URDF\urdf\robot_arm_URDF.urdf",startPos, startOrientation, fixedBase)
#boxId = p.loadURDF(r"C:\Users\12105\OneDrive\Documents\MEEN 401\hello\new_robot_urdf_5\urdf\new_robot_urdf_5.urdf",startPos, startOrientation, fixedBase)
p.resetBasePositionAndOrientation(boxId, [0, 0, 0], [0, 0, 0, 1])
joint_ids = []

#robot properties
maxVel = 3.703 #rad/s
for j in range(p.getNumJoints(boxId)):
    p.changeDynamics( boxId, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
    p.changeDynamics( boxId, j, maxJointVelocity=maxVel)
    joint_ids.append( p.getJointInfo(boxId, j))

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)


#makes list of joints and joint info, useful for indexing
# nJoints = p.getNumJoints(boxId)
# print(nJoints)
# jointNameToId = {}
# for i in range(nJoints):
#   jointInfo = p.getJointInfo(boxId, i)
#   jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
# print(jointInfo)
# print(jointNameToId)

# joint_1 = jointNameToId['joint_1']
# joint_2 = jointNameToId['joint_2']
# joint_3 = jointNameToId['joint_3']
# joint_4 = jointNameToId['joint_4']
p.createConstraint(planeId, -1, boxId, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0]  )
p.createConstraint(boxId, 4, boxId, 5, p.JOINT_FIXED, [0,0,0], [0,0.,0], [0.0,0,0]  )
p.createConstraint(boxId, 4, boxId, 6, p.JOINT_FIXED, [0,0,0], [0.,0,0], [0.0,0,0]  )
from time import sleep

from dualsense_controller import DualSenseController
# list availabe devices and throw exception when tzhere is no device detected
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')
# flag, which keeps program alive
#is_running = True
controller = DualSenseController()
controller.activate()
# this is bad practice, but we need these variables to initialize and maintain value throughout
# the run simulation so making global variables allows us to initialize and then continuously update
# variables throughout the sim
left_stick_y_global = 1
left_stick_x_global = 1
right_stick_y_global = 1
right_stick_x_global = 1
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

def on_right_stick_y_changed(right_stick_y):
        global right_stick_y_global
        right_stick_y_global = right_stick_y
        print(right_stick_y_global)
# callback when legt joystick is moved in x direction
def on_right_stick_x_changed(right_stick_x):
        global right_stick_x_global
        right_stick_x_global = right_stick_x
        print(right_stick_x_global)
# calls function from the dualsense_controller library that reads controller input
# when function senses specified controller input, it calls the callback function that updates the global variables
controller.left_stick_x.on_change(on_left_stick_x_changed)
controller.left_stick_y.on_change(on_left_stick_y_changed)
controller.right_stick_x.on_change(on_right_stick_x_changed)
controller.right_stick_y.on_change(on_right_stick_y_changed)



for i in range (100000):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=90, cameraPitch=-15, cameraTargetPosition=[0,0,0]) #-pitch get view higher in the sky
    maxForce = 200 #N/m
    N_steps=500000
    
    # create an instance, use fiŕst available device, in this case we're using usb connected device
    
    angles = [left_stick_x_global*3.14, right_stick_x_global*3.14, left_stick_y_global*-3.14, right_stick_y_global*3.14]
    #angles = [0,0,0,0]
    print(angles)
    p.setJointMotorControl2(boxId, 0, p.POSITION_CONTROL, targetPosition = angles[0] , force = maxForce)
    p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition = angles[1] , force = maxForce)
    p.setJointMotorControl2(boxId, 2, p.POSITION_CONTROL, targetPosition = angles[2] , force = maxForce)
    p.setJointMotorControl2(boxId, 3, p.POSITION_CONTROL, targetPosition = angles[3] , force = maxForce)
    p.setJointMotorControl2(boxId, 4, p.POSITION_CONTROL, targetPosition = 1 , force = maxForce)
    #p.setJointMotorControl2(boxId, 5, p.POSITION_CONTROL, targetPosition = 1 , force = maxForce)
    #p.setJointMotorControl2(boxId, 6, p.POSITION_CONTROL, targetPosition = 1 , force = maxForce)
    p.stepSimulation()
    time.sleep(1./240.)
#cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
