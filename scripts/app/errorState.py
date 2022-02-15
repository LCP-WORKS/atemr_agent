#!/usr/bin/env python3

import rospy
import smach
from utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes

'''
    make call to reset motors should such error occur
'''

class ERRORState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['err_obj_i'], output_keys=['shutdown_action_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData(akeys.SM_STATE, astates.ERR))
        rospy.init_node('sm_err_node')

    def execute(self, userdata):
        rospy.loginfo('Error ...')
        rate = rospy.Rate(1)
        cnt = 0

        while(not rospy.is_shutdown()):
            rospy.loginfo("ERROR running ....")
            rate.sleep()
            cnt += 1
            if(cnt > 3):
                break

        return 'success'