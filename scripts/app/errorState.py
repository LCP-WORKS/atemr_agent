#!/usr/bin/env python3

import rospy
import smach
from app.utils.helper import StateData

'''
    make call to reset motors should such error occur
'''

class ERRORState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['error_obj'], output_keys=['error_obj'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.out_queue.put(StateData('sm_state', 'ERROR'))

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