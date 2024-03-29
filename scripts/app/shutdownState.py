#!/usr/bin/env python3

import os
import rospy
import smach
from atemr_msgs.srv import HardwareService, HardwareServiceRequest
from atemr_msgs.srv import NodeControllerService, NodeControllerServiceRequest
from app.utils.helper import StateData, AgentStates as astates, ShutdownAction, \
                            AgentKeys as akeys

class SHUTDOWNState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'restart'], input_keys=['shutdown_action_i'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.hdwServer = rospy.ServiceProxy('HARDWAREServer', HardwareService)

    def execute(self, userdata):
        rospy.loginfo('Shutdown ...')
        action = userdata.shutdown_action_i
        outcome = 'success'
        node_ctlClient = rospy.ServiceProxy('NodeControllerServer', NodeControllerService)

        self.out_queue.put(StateData(akeys.SM_STATE, astates.SDWN))
        while(not rospy.is_shutdown()):
            rospy.loginfo("SHUTDOWN running ....")

            #kill all launches
            try:
                rospy.wait_for_service('NodeControllerServer', timeout=5)
                req = NodeControllerServiceRequest()
                req.is_basics.data = True
                req.basics_action.data = False #terminate
                node_ctlClient.call(req)
            except rospy.ROSException as e:
                rospy.logerr(e)
            
            if((action.value == ShutdownAction.SHUTDOWN.value) or (action == None)): #make service call to shutdown PC
                os.system('sudo shutdown now -h')
                """ try:
                    req = HardwareServiceRequest()
                    req.shutdownSystem.data = True
                    resp = self.hdwServer(req)
                    if(resp.result.data):
                        break
                    else:
                        rospy.logerr('Failed to trigger shutdown')
                except (rospy.ServiceException, rospy.ROSException) as e:
                    rospy.logerr('Service call failed: %s' % e) """
            if((action.value == ShutdownAction.RESTART.value)): #make service call to restart PC
                os.system('sudo reboot')
                """ try:
                    req = HardwareServiceRequest()
                    req.restartSystem.data = True
                    resp = self.hdwServer(req)
                    if(resp.result.data):
                        break
                    else:
                        rospy.logerr('Failed to trigger restart')
                except (rospy.ServiceException, rospy.ROSException) as e:
                    rospy.logerr('Service call failed: %s' % e) """

        return outcome