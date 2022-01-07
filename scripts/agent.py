import app.startupState
import app.idleState
import app.execState
import app.mappingState
import app.errorState
import app.shutdownState
import app.monitorState

import smach
from multiprocessing import Queue
from queue import Empty

class CQueue(Queue):
    def clear(self):
        try:
            while True:
                self.get_nowait()
        except Empty:
            pass

def main():
    states_to_monitor_queue = CQueue()
    monitor_to_states_queue = CQueue()

    # Create the sub SMACH state machine
    sm_con = smach.Concurrence(outcomes=['complete'],
                                default_outcome='complete',
                                outcome_map={'complete':
                                                {'RUNNING_SM':'exit',
                                                'MONITOR':'success'}},
                                input_keys=['parent_counter'],
                                output_keys=['parent_counter'])
    sm_con.userdata.parent_counter = 0
    # Open the container
    with sm_con:
        # Add states to the concurrency container

        #create mid level SMACH state machine
        sm_run = smach.StateMachine(outcomes=['exit'])
        # Add states to the mid level container
        with sm_run:
            smach.StateMachine.add('STARTUP', app.startupState.STARTUPState(outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure': 'SHUTDOWN'})
            smach.StateMachine.add('IDLE', app.idleState.IDLEState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'nav':'EXECUTION',
                                        'map':'MAPPING',
                                        'err':'ERROR',
                                        'terminate': 'exit'})
            smach.StateMachine.add('EXECUTION', app.execState.EXECState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure':'ERROR'})
            smach.StateMachine.add('MAPPING', app.mappingState.MAPState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure':'ERROR'})
            smach.StateMachine.add('ERROR', app.errorState.ERRORState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'IDLE',
                                        'failure':'exit'})
            smach.StateMachine.add('SHUTDOWN', app.shutdownState.SHUTDOWNState(incoming_queue=monitor_to_states_queue, outgoing_queue=states_to_monitor_queue),
                            transitions={'success':'complete',
                                        'restart': 'STARTUP'})


        smach.Concurrence.add('MONITOR', app.monitorState.MONITORState(incoming_queue=states_to_monitor_queue, outgoing_queue=monitor_to_states_queue), 
                            remapping={'bar_counter':'parent_counter'},
                            transitions={'success':'MONITOR'}) #monitoring state goes here
        smach.Concurrence.add('RUNNING_SM', sm_run, remapping={'foo_counter':'parent_counter'})

    # Execute SMACH plan
    outcome = sm_con.execute()


if __name__ == '__main__':
    main()
