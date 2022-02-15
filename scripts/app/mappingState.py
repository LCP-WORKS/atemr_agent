#!/usr/bin/env python3

import rospy
import smach
from app.utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes

'''
    map_action:
        - change map     -> 0
        - create new map -> 1
'''

class MAPState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['map_data_i'], output_keys=['err_obj_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData(akeys.SM_STATE, astates.MAP))
        rospy.init_node('sm_map_node')

    def execute(self, userdata):
        rospy.loginfo('Mapping ...')
        rate = rospy.Rate(1)
        cnt = 0

        #check if rloc_world is started else start it

        while(not rospy.is_shutdown()):
            rospy.loginfo("MAP running ....")
            rate.sleep()
            cnt += 1
            if(cnt > 3):
                break

        return 'failure'