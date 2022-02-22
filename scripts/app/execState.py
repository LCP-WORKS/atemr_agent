#!/usr/bin/env python3

import rospy
import smach
from app.utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes
from actionlib_msgs.msg import GoalStatus

class EXECState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['goal_obj_i'], output_keys=['err_obj_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.nav_result = None
        self.is_navigating = True

    def monitor_goal(self, msg):
        #monitor goal
        if((msg == GoalStatus.ACTIVE)):
            self.is_navigating = True
        if((msg == GoalStatus.SUCCEEDED)):
            self.nav_result = 'success'
            return True
        if((msg == GoalStatus.ABORTED) or (msg == GoalStatus.REJECTED) or (msg == GoalStatus.RECALLED)):
            self.nav_result = 'failure'
            return True
        return False

    def execute(self, userdata):
        rospy.loginfo('Execution ...')
        outcome = 'failure'
        rate = rospy.Rate(15)

        self.out_queue.put(StateData(akeys.SM_STATE, astates.EXC))
        while(not rospy.is_shutdown()):
            rospy.loginfo_throttle(1, "EXEC running ....")
            if(not self.in_queue.empty()):
                msg_obj = sdataDecoder(self.in_queue, astates.EXC)
                if(msg_obj is not None): #process received queue data here
                    if(msg_obj.name == akeys.TRIGR_STATE):
                        sm_state = msg_obj.dataObject
                        if(sm_state == astates.IDL):
                            outcome = 'success'
                            break
                        if(sm_state == astates.ERR):
                            outcome = 'failure'
                            break
                    if(msg_obj.name == akeys.TRIGR_STATE_EXTRA):
                        (sm_state, data)= msg_obj.dataObject
                        if(sm_state == astates.EXC):
                            if(self.monitor_goal(data)):
                                outcome = self.nav_result
                                break
                    else:
                        rospy.logwarn('Received unknown from queue: %s' % msg_obj.name)
                msg_obj = None

            #check if goal reached or terminated and transition
            if(self.is_navigating):
                rospy.loginfo_throttle(1, 'Is navigating ...')
            rate.sleep()
        
        return outcome