#!/usr/bin/env python3
 
import rospy
import smach
import pickle
from app.utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes

class IDLEState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['nav', 'map', 'err', 'sdown'],
                                   output_keys=['goal_obj_o', 'map_data_o', 'err_obj_o', 'shutdown_action_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue

    def execute(self, userdata):
        rospy.loginfo('Idling ...')
        rate = rospy.Rate(20)
        outcome = None
        #userdata.launch_obj = None
        userdata.goal_obj_o = None
        userdata.map_data_o = None
        userdata.err_obj_o = None
        userdata.shutdown_action_o = None
        
        self.out_queue.put(StateData(akeys.SM_STATE, astates.IDL))
        # Trigger successful startup acknowledgement
        #self.out_queue.put(StateData(akeys.TRIGR_ACK, True)) #UNCOMMENT WHEN RUNNING ON REAL
        while(not rospy.is_shutdown()):
            rospy.loginfo_throttle(3, "IDLE running ....")
            #check and process incoming data
            if(not self.in_queue.empty()):
                msg_obj = sdataDecoder(self.in_queue, astates.IDL)
                if(msg_obj is not None): #process received queue data here
                    if(msg_obj.name == akeys.TRIGR_STATE_EXTRA):
                        (sm_state, data) = msg_obj.dataObject
                        #process mapping
                        if(sm_state == astates.MAP):
                            rospy.loginfo(data[0].value)
                            userdata.map_data_o = data
                            outcome = 'map'
                            break
                        #process shutdown
                        if(sm_state == astates.SDWN):
                            userdata.shutdown_action_o = data[0]
                            #userdata.launch_obj = data[1]
                            outcome = 'sdown'
                            break
                        #process error
                        if(sm_state == astates.ERR):
                            userdata.err_obj_o = data
                            outcome = 'err'
                            break
                    elif(msg_obj.name == akeys.TRIGR_STATE):
                        sm_state = msg_obj.dataObject
                        #process navigation
                        if(sm_state == astates.EXC):
                            outcome = 'nav'
                            break
                        #process error
                        if(sm_state == astates.ERR):
                            outcome = 'err'
                            break
                        else:
                            rospy.logwarn('Received unknown from queue: %s' % msg_obj.name)
                    else:
                        rospy.logwarn('Received unknown from queue: %s' % msg_obj.name)
                msg_obj = None


            rate.sleep()
        if(outcome == None): #catch limbo fallthroughs
            outcome = 'run_complete' # change to 'err' when running real
            self.in_queue.put(StateData(akeys.ERR_OBJ, 
                                        (ErrCodes.STATE_FALLTHROUGH, 'Unstable state: IDLE'), astates.ERR))
        return outcome