#!/usr/bin/env python3

import rospy
import smach
from app.utils.helper import StateData

'''
    map_action:
        - change map     -> 0
        - create new map -> 1
'''

class MAPState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['map_action'], output_keys=['error_obj'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData('sm_state', 'MAPPING'))

    def execute(self, userdata):
        rospy.loginfo('Mapping ...')
        rate = rospy.Rate(1)
        cnt = 0

        while(not rospy.is_shutdown()):
            rospy.loginfo("MAP running ....")
            rate.sleep()
            cnt += 1
            if(cnt > 3):
                break

        return 'failure'