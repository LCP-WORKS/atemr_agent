#!/usr/bin/env python3

import time
import rospy
import smach

from bitarray import bitarray
from app.utils.checkCAN import checkAndActivateCANInterface
from app.utils.robot_launcher import RobotLauncher
from app.utils.helper import StateData, AgentKeys as akeys, AgentStates as astates, ShutdownAction

class STARTUPState(smach.State):
    def __init__(self, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                    output_keys=['launch_obj', 'shutdown_action'])
        self.launcher = RobotLauncher()
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData(akeys.SM_STATE, astates.SUP))

    def execute(self, userdata):
        rospy.loginfo('Starting up ...')
        cnt = 0 
        max_retries = 3
        '''
            Indices ->     0     |     1      |  2  |   3   |    4   |     5     |        6       |        7      |  8
            Repr    -> LeftMotor | RightMotor | IMU | Lidar | Camera | WebServer | IRSensor Front | IRSensor Rear | CAN
        '''
        module_states = bitarray(9)
        module_states.setall(0)
        tmp_states = bitarray(4)
        tmp_states.setall(0)
        outcome = None
        userdata.launch_obj = None
        userdata.shutdown_action = ShutdownAction.SHUTDOWN
        while(True):
            print("STARTUP running ....")
            #CAN Interface
            module_states[8] = 1 #auto set CAN interface
            #if(module_states[8] == 0):
                #module_states[8] = 1 if(checkAndActivateCANInterface()) else 0
            
            #launch base with sensors
            tmp_states = self.launcher.run(module_states=tmp_states)
            if(tmp_states.all() and module_states[8]):
                module_states[2:5] = tmp_states[1:]
                outcome = 'success'
                break
            else:
                if(cnt >= max_retries):
                    print('Retries exhausted !!')
                    outcome = 'failure'
                    break
                print('Modules initialization failure. Retry in a few seconds ...')
                self.launcher.terminate(module_states=tmp_states, isRetry=True)
                cnt += 1
            
            time.sleep(3.0) #wait another 3 seconds before trying to restart the modules
        
        #userdata.module_states = module_states[:7]
        userdata.launch_obj = self.launcher
        self.out_queue.put(StateData(akeys.LNCH_OBJ, self.launcher))
        self.out_queue.put(StateData(akeys.MOD_STATES, tmp_states))
        return outcome