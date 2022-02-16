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
    
    def cancelGoal(self):
        pass

    def execute(self, userdata):
        rospy.loginfo('Execution ...')
        outcome = None
        rate = rospy.Rate(15)

        while(not rospy.is_shutdown()):
            rospy.loginfo("EXEC running ....")
            if(not self.in_queue.empty()):
                msg_obj = sdataDecoder(self.in_queue, astates.EXC)
                if(msg_obj is not None): #process received queue data here
                    if(msg_obj.name == akeys.TRIGR_STATE_EXTRA):
                        (sm_state, data) = msg_obj.dataObject
                        #process goal commands
                        if(sm_state == astates.IDL):
                            if(data.data):
                                self.cancelGoal()
                                outcome = 'success'
                                break
                    else:
                        rospy.logwarn('Received unknown from queue: %s' % msg_obj.name)

            #check if goal reached or terminated and transition
            rate.sleep()

        return outcome