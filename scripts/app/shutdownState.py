#!/usr/bin/env python3

import time
import rospy
import smach
from atemr_msgs.srv import HardwareService, HardwareServiceRequest
from utils.helper import StateData, AgentStates as astates, ShutdownAction, \
                            AgentKeys as akeys

class SHUTDOWNState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'restart'], input_keys=['shutdown_action_i', 'launch_obj_i'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData(akeys.SM_STATE, astates.SDWN))
        rospy.init_node('sm_shutdown_node')

    def execute(self, userdata):
        rospy.loginfo('Shutdown ...')
        action = userdata.shutdown_action_i
        launch = userdata.launch_obj_i

        while(True):
            rospy.loginfo("SHUTDOWN running ....")
            if((action == ShutdownAction.SHUTDOWN) or (action == None)):
                #make service call to shutdown
                try:
                    hdwServer = rospy.ServiceProxy('HARDWAREServer', HardwareService)
                    req = HardwareServiceRequest()
                    req.shutdownSystem.data = True
                    if(hdwServer(req)):
                        time.sleep(4.0)
                except rospy.ServiceException as e:
                    print('Service call failed: %s' % e)

            #kill all launches
            launch.terminate()
            if(action == ShutdownAction.RESTART):
                time.sleep(5.0)
                outcome = 'restart'
                break
            else:
                outcome = 'success'
                break
        return outcome