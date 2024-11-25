# import pybullet as p
# import time
# import numpy as np
# import sys

# class pybulletDebug:
#     def __init__(self):
#         #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
#         self.cyaw=90
#         self.cpitch=-7
#         self.cdist=0.30
#         time.sleep(0.5)
        
#         self.xId = p.addUserDebugParameter("x" , -0.02 , 0.02 , 0.)
#         self.yId = p.addUserDebugParameter("y" , -0.02 , 0.02 , 0.)
#         self.zId = p.addUserDebugParameter("z" , -0.02 , 0.02 , 0.)
#         self.rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
#         self.pitchId = p.addUserDebugParameter("pitch" , -np.pi/4 , np.pi/4 , 0.)
#         self.yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)
#         self.LId = p.addUserDebugParameter("L" , -0.50 , 1 , 0.)
#         self.LrotId = p.addUserDebugParameter("Lrot" , -1.50 , 1.50 , 0.)
#         self.angleId = p.addUserDebugParameter("angleWalk" , -180. , 180. , 0.)
#         self.periodId = p.addUserDebugParameter("stepPeriod" , 0.1 , 3. , 1.0)
#         self.step_dur_asym = p.addUserDebugParameter("step_dur_asym" , -2 , 2. , 0.0)
#         self.trotId = p.addUserDebugParameter("TROT" , 1 , 0 , 1)
#         self.boundId = p.addUserDebugParameter("BOUND" , 1 , 0 , 1)
        
    
#     def cam_and_robotstates(self , boxId):
#         #orientation of camara
#         cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#         p.resetDebugVisualizerCamera( cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=cubePos)
#         keys = p.getKeyboardEvents()
#         #Keys to change camera
#         if keys.get(104):  #H
#             self.cyaw+=1
#         if keys.get(102):  #F
#             self.cyaw-=1
#         if keys.get(116):  #T
#             self.cpitch+=1
#         if keys.get(103):  #G
#             self.cpitch-=1
#         if keys.get(122):  #Z
#             self.cdist+=0.01
#         if keys.get(120):  #X
#             self.cdist-=0.01
#         if keys.get(27):  #ESC
#             p.disconnect()
#             time.sleep(2)
            
#         #   sys.exit()
#         #read position from debug
#         pos = np.array([p.readUserDebugParameter(self.xId),p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
#         orn = np.array([p.readUserDebugParameter(self.rollId),p.readUserDebugParameter(self.pitchId), p.readUserDebugParameter(self.yawId)])
#         L = p.readUserDebugParameter(self.LId)
#         Lrot = p.readUserDebugParameter(self.LrotId)
#         angle = p.readUserDebugParameter(self.angleId)
#         T = p.readUserDebugParameter(self.periodId)
#         trot=p.readUserDebugParameter(self.trotId)
#         bound=p.readUserDebugParameter(self.boundId)
        
#         if trot==1:
#           offset=[0.5, 0., 0., 0.5]
#         elif bound==1:
#           offset=[0.5, 0.5, 0., 0.]
#         else:
#           offset=[0.5, 0., 0., 0.5]
        
        
#         return pos , orn , L , angle , Lrot , T , p.readUserDebugParameter(self.step_dur_asym), offset


import pybullet as p
import time
import numpy as np
import sys

# implement ps5 controller
from pydualsense import pydualsense, TriggerModes
class pybulletDebug:
    
    
    
    def __init__(self):
        #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
        from time import sleep

        from dualsense_controller import DualSenseController

        # list availabe devices and throw exception when tzhere is no device detected
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception('No DualSense Controller available.')

        # flag, which keeps program alive
        #is_running = True

        # create an instance, use fiŕst available device
        controller = DualSenseController()


        # switches the keep alive flag, which stops the below loop
        # def stop():
        #     global is_running
        #     is_running = False


        # # callback, when cross button is pressed, which enables rumble
        # def on_cross_btn_pressed():
        #     global count
        #     print('cross button pressed')
        #     controller.left_rumble.set(255)
        #     controller.right_rumble.set(255)
        #     count += 1
        #     print(count)
            


        # # callback, when cross button is released, which disables rumble
        # def on_cross_btn_released():
        #     global count
        #     print('cross button released')
        #     controller.left_rumble.set(0)
        #     controller.right_rumble.set(0)
        #     count += 1
        #     print(count)
            


        # # callback, when PlayStation button is pressed
        # # stop program
        # def on_ps_btn_pressed():
        #     print('PS button released -> stop')
        #     stop()


        # # callback, when unintended error occurs,
        # # i.e. physically disconnecting the controller during operation
        # # stop program
        # def on_error(error):
        #     print(f'Opps! an error occured: {error}')
        #     stop()

        # def on_left_trigger(value):
        #     print(f'left trigger changed: {value}')


        # def on_left_stick_x_changed(left_stick_x):
        #     print(f'on_left_stick_x_changed: {left_stick_x}')


        def on_left_stick_y_changed(left_stick_y):
            global left_stick_y_global
            #print(f'on_left_stick_y_changed: {left_stick_y}')
            #print(type(left_stick_y))
            self.left_stick_y_global = left_stick_y
            print(self.left_stick_y_global)

        
        self.left_stick_y_global = 0
        # controller.left_trigger.on_change(on_left_trigger)
        # controller.left_stick_x.on_change(on_left_stick_x_changed)
        controller.left_stick_y.on_change(on_left_stick_y_changed)

        # def on_left_stick_changed(left_stick):
        #     print(f'on_left_stick_changed: {left_stick}')
        #     #print(type(left_stick))
        #     #print({left_stick})


        # controller.left_trigger.on_change(on_left_trigger)
        # controller.left_stick_x.on_change(on_left_stick_x_changed)
        # controller.left_stick_y.on_change(on_left_stick_y_changed)
        # controller.left_stick.on_change(on_left_stick_changed)
        # count = 0
        # # register the button callbacks
        # controller.btn_cross.on_down(on_cross_btn_pressed)
        # controller.btn_cross.on_up(on_cross_btn_released)
        # controller.btn_ps.on_down(on_ps_btn_pressed)

        # register the error callback
        #controller.on_error(on_error)

        # enable/connect the device
        controller.activate()

        # start keep alive loop, controller inputs and callbacks are handled in a second thread
        # while is_running:
        #     sleep(0.001)

        # disable/disconnect controller device
        controller.deactivate()
        self.cyaw=90
        self.cpitch=-7
        self.cdist=0.7
        time.sleep(0.5)

        self.xId = p.addUserDebugParameter("x" , -0.1 , 0.02 , 0)
        self.yId = p.addUserDebugParameter("y" , -0.02 , 0.02 , 0.)
        self.zId = p.addUserDebugParameter("z" , -0.1 , 0.05 , 0)
        self.rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
        self.pitchId = p.addUserDebugParameter("pitch" , -np.pi/2 , np.pi/4 , 0)
        self.yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)
        self.LId = p.addUserDebugParameter("L" , -0.50 , 1 , 0.)
        self.LrotId = self.left_stick_y_global
        print(self.LrotId)
        self.angleId = 0
        self.periodId = p.addUserDebugParameter("stepPeriod" , 0.1 , 3. , 1.0)
        self.step_dur_asym = 0
        self.trotId = p.addUserDebugParameter("TROT" , 1 , 0 , 1)
        self.boundId = p.addUserDebugParameter("BOUND" , 1 , 0 , 1)
        
    
    def cam_and_robotstates(self , boxId):
        #orientation of camara
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        p.resetDebugVisualizerCamera( cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=cubePos)
        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(104):  #H
            self.cyaw+=1
        if keys.get(102):  #F
            self.cyaw-=1
        if keys.get(116):  #T
            self.cpitch+=1
        if keys.get(103):  #G
            self.cpitch-=1
        if keys.get(122):  #Z
            self.cdist+=0.01
        if keys.get(120):  #X
            self.cdist-=0.01
        if keys.get(27):  #ESC
            p.disconnect()
            time.sleep(2)
            
        #   sys.exit()
        #read position from debug
        pos = np.array([p.readUserDebugParameter(self.xId),p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
        
        orn = np.array([p.readUserDebugParameter(self.rollId),p.readUserDebugParameter(self.pitchId), p.readUserDebugParameter(self.yawId)])
        L = p.readUserDebugParameter(self.LId)
        Lrot = self.LrotId

        angle = p.readUserDebugParameter(self.angleId)
        T = p.readUserDebugParameter(self.periodId)
        #trot=1
        trot=p.readUserDebugParameter(self.trotId)
        bound=p.readUserDebugParameter(self.boundId)
        
        if trot==1:
          offset=[0.5, 0., 0., 0.5]
        elif bound==1:
          offset=[0.5, 0.5, 0., 0.]
        else:
          offset=[0.5, 0., 0., 0.5]
        
        
        return pos , orn , L , angle , Lrot , T , p.readUserDebugParameter(self.step_dur_asym), offset
