#!/usr/bin/env python3

import time
import rospy
import smach

from bitarray import bitarray
from std_msgs.msg import String
from atemr_msgs.srv import NodeControllerService, NodeControllerServiceRequest
from app.utils.config import cfgContext
from app.utils.robot_launcher import NodeType as NT
#from app.utils.checkCAN import checkAndActivateCANInterface
from app.utils.helper import StateData, AgentKeys as akeys, AgentStates as astates, ShutdownAction
from bitarray.util import int2ba

class STARTUPState(smach.State):
    def __init__(self, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                    output_keys=['shutdown_action_o'])
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
        node_states = bitarray(7)
        node_states.setall(0)
        outcome = None
        nctlr_ready = False
        node_ctlClient = rospy.ServiceProxy('NodeControllerServer', NodeControllerService)

        #UNCOMMENT FOR TESTING
        #nctlr_ready = True
        #node_states.setall(1)
        
        while(not rospy.is_shutdown()):
            rospy.loginfo("STARTUP running ....")
            #CAN Interface
            module_states[8] = 1 #auto set CAN interface
            #if(module_states[8] == 0):
                #module_states[8] = 1 if(checkAndActivateCANInterface()) else 0

            if(nctlr_ready):
                try:
                    rospy.wait_for_service('NodeControllerServer', timeout=5)#COMMENT FOR TESTING
                    # get hardware status by making an empty call to the server
                    resp = node_ctlClient.call(NodeControllerServiceRequest())
                    node_states = int2ba(resp.hardwareStatus)
                    #rospy.loginfo(node_states)

                    #launch base with sensors (124 ->  1111100)
                    if((resp.hardwareStatus == 124) and module_states[8]):
                        #module_states[2:5] = tmp_states[1:]
                        outcome = 'success'
                        break
                    else:
                        if(cnt >= max_retries):
                            rospy.logerr('Retries exhausted !!')
                            outcome = 'failure'
                            break
                        rospy.loginfo('Modules initialization failure. Retry in a few seconds ...')
                        #make call to restart basis
                        rospy.wait_for_service('NodeControllerServer', timeout=5)
                        req = NodeControllerServiceRequest()
                        req.is_basics.data = True
                        req.basics_action.data = True #restart
                        resp = node_ctlClient.call(req)
                        node_states = bitarray(resp.hardwareStatus)
                except (rospy.ROSException, rospy.ServiceException) as e:
                    rospy.logerr(e)
                cnt += 1
                time.sleep(3.0) #wait another 3 seconds before trying to restart the modules
            else: #check for READY message from NodeController
                try:
                    msg = rospy.wait_for_message(cfgContext['node_controller_topic'], String, timeout=10)
                    if(msg.data == 'READY'):
                        nctlr_ready = True
                except rospy.ROSException as e:
                    print(e)
        
        #userdata.module_states = module_states[:7]
        userdata.shutdown_action_o = ShutdownAction.SHUTDOWN
        #self.out_queue.put(StateData(akeys.LNCH_OBJ, self.launcher))
        self.out_queue.put(StateData(akeys.MOD_STATES, node_states))
        return outcome