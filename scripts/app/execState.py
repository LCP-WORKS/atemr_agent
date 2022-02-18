#!/usr/bin/env python3

import rospy
import smach
from app.utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction
import actionlib

class EXECState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['goal_obj_i'], output_keys=['err_obj_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        self.nav_result = None
        self.is_navigating = True

    def monitor_goal(self, msg):
        #monitor feedback
        if((msg[0].status == GoalStatus.ACTIVE)):
            return
        if((msg[0].status == GoalStatus.SUCCEEDED)):
            self.nav_result = 'success'
            return
        if((msg[0].status == GoalStatus.ABORTED) or (msg[0].status == GoalStatus.REJECTED)):
            self.nav_result = 'failure'
            return

    def execute(self, userdata):
        rospy.loginfo('Execution ...')
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_monitor_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.monitor_goal)
        outcome = None
        rate = rospy.Rate(15)

        self.out_queue.put(StateData(akeys.SM_STATE, astates.EXC))
        self.mb_client.wait_for_server()
        while(not rospy.is_shutdown()):
            rospy.loginfo_throttle(1, "EXEC running ....")
            if(not self.in_queue.empty()):
                msg_obj = sdataDecoder(self.in_queue, astates.EXC)
                if(msg_obj is not None): #process received queue data here
                    if(msg_obj.name == akeys.TRIGR_STATE_EXTRA):
                        (sm_state, data) = msg_obj.dataObject
                        #process goal commands
                        if(sm_state == astates.IDL):
                            if(data):
                                self.mb_client.cancel_all_goals()
                                outcome = 'success'
                                break
                    else:
                        rospy.logwarn('Received unknown from queue: %s' % msg_obj.name)
                msg_obj = None

            #check if goal reached or terminated and transition
            if(not self.is_navigating):
                rospy.loginfo('Point reached!')
                outcome = self.nav_result
                break
            else:
                rospy.loginfo_throttle('Is navigating ...')
            rate.sleep()

        self.goal_monitor_sub.unregister()
        return outcome