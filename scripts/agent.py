#!/usr/bin/env python3

import app.startupState
import app.idleState
import app.execState
import app.mappingState
import app.errorState
import app.shutdownState
import app.monitorState
import rospy

import smach
from multiprocessing import Queue
from queue import Empty

#class CQueue(Queue):
#    def clear(self):
#        try:
#            while True:
#                self.get_nowait()
#        except Empty:
#            pass

def main():
    states_to_monitor_queue = Queue()
    monitor_to_states_queue = Queue()

    # Create the sub SMACH state machine
    sm_con = smach.Concurrence(outcomes=['complete'],
                                default_outcome='complete',
                                outcome_map={'complete':
                                                {'RUNNING_SM':'run_complete',
                                                'MONITOR':'success'}})
    # Open the container
    with sm_con:
        # Add states to the concurrency container

        #create mid level SMACH state machine
        sm_run = smach.StateMachine(outcomes=['run_complete'])
        sm_run.userdata.sm_launch_obj = None
        sm_run.userdata.sm_goal_obj = None
        sm_run.userdata.sm_map_data = None
        sm_run.userdata.sm_err_obj = None
        sm_run.userdata.sm_shutdown_action = None
        # Add states to the mid level container
        with sm_run:
            smach.StateMachine.add('STARTUP', app.startupState.STARTUPState(outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure': 'SHUTDOWN'},
                            remapping={'shutdown_action_o' : 'sm_shutdown_action', 
                                        'launch_obj_o' : 'sm_launch_obj'})
            smach.StateMachine.add('IDLE', app.idleState.IDLEState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'nav':'EXECUTION',
                                        'map':'MAPPING',
                                        'err':'ERROR',
                                        'sdown': 'SHUTDOWN'},
                            remapping={ 'goal_obj_o' : 'sm_goal_obj',
                                        'map_data_o' : 'sm_map_data',
                                        'err_obj_o' : 'sm_err_obj', 
                                        'shutdown_action_o' : 'sm_shutdown_action'})
            smach.StateMachine.add('EXECUTION', app.execState.EXECState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure':'ERROR'},
                            remapping={ 'goal_obj_i' : 'sm_goal_obj',
                                        'err_obj_o' : 'sm_err_obj'})
            smach.StateMachine.add('MAPPING', app.mappingState.MAPState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure':'ERROR'},
                            remapping={ 'map_data_i' : 'sm_map_data',
                                        'err_obj_o' : 'sm_err_obj'})
            smach.StateMachine.add('ERROR', app.errorState.ERRORState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure':'SHUTDOWN'},
                            remapping={ 'err_obj_i' : 'sm_err_obj',
                                        'shutdown_action_o' : 'sm_shutdown_action'})
            smach.StateMachine.add('SHUTDOWN', app.shutdownState.SHUTDOWNState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'run_complete',
                                        'restart': 'STARTUP'},
                            remapping={'shutdown_action_i' : 'sm_shutdown_action',
                                        'launch_obj_i' : 'sm_launch_obj'})


        smach.Concurrence.add('RUNNING_SM', sm_run)
        smach.Concurrence.add('MONITOR', app.monitorState.MONITORState(incoming_queue=states_to_monitor_queue, outgoing_queue=monitor_to_states_queue)) #monitoring state goes here

    # Execute SMACH plan
    outcome = sm_con.execute()


if __name__ == '__main__':
    main()
