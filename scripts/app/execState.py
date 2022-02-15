#!/usr/bin/env python3

import rospy
import smach
from app.utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes

class EXECState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['goal_obj_i'], output_keys=['err_obj_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData(akeys.SM_STATE, astates.EXC))
        rospy.init_node('sm_exec_node')

    def execute(self, userdata):
        rospy.loginfo('Execution ...')
        rate = rospy.Rate(1)
        cnt = 0

        while(not rospy.is_shutdown()):
            rospy.loginfo("EXEC running ....")
            rate.sleep()
            cnt += 1
            if(cnt > 3):
                break

        return 'success'