#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from zasState import Zas
from multiprocessing import Process
from queue import Queue, Empty

global_var = 0

class ClearableQueue(Queue):

    def clear(self):
        try:
            while True:
                self.get_nowait()
        except Empty:
            pass

# define state Foo
class Foo(smach.State):
    def __init__(self, state_queue):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'], input_keys=['foo_counter'], output_keys=['foo_counter', 'third_hypo'])
        self.counter = 0
        self.state_queue = state_queue

    def execute(self, userdata):
        global global_var
        rospy.loginfo('Executing state FOO')
        rate = rospy.Rate(1)
        userdata.third_hypo = 'Freaking works!'
        while(not rospy.is_shutdown()):
            rospy.loginfo("Foo running ....%i" % global_var)
            userdata.foo_counter += 1
            global_var += 1
            self.state_queue.put(global_var)
            if(userdata.foo_counter > 30 or global_var > 10):
                break
            rate.sleep()
        return 'outcome2'
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome3'], input_keys=['bar_counter'], output_keys=['bar_counter'])

    def execute(self, userdata):
        global global_var
        rospy.loginfo('Executing state BAR')
        rate = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            rospy.loginfo("Bar running ....")
            rate.sleep()
            if(userdata.bar_counter == 5 or global_var == 5):
                return 'outcome3'
            if(userdata.bar_counter > 30):
                break

        return 'outcome1'

# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        return 'outcome3'




def main():
    rospy.init_node('smach_example_state_machine')
    state_queue = ClearableQueue()

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome6'])
    sm_top.userdata.parent_counter = 4
    sm_top.userdata.data_out_foo = 'Hellooo'

    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'CON'})

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['outcome4','outcome5'],
                                   default_outcome='outcome4',
                                   outcome_map={'outcome5':
                                                { 'FOO':'outcome2',
                                                  'MID_SM':'outcome6'}},
                                    input_keys=['parent_counter', 'data_out_foo'], output_keys=['parent_counter', 'data_out_foo'])

        # Open the container
        with sm_con:
            # Add states to the container

            #mid level container
            sm_mid = smach.StateMachine(outcomes=['outcome6'], input_keys=['mid_input', 'my_third'])
            with sm_mid:
                # Add states
                smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome3':'ZAS', 'outcome1' : 'outcome6'},
                               remapping={'bar_counter':'mid_input'})
                smach.StateMachine.add('ZAS', Zas(state_queue=state_queue), remapping={'third_hypo_zas' : 'my_third'})

            smach.Concurrence.add('FOO', Foo(state_queue=state_queue), remapping={'foo_counter':'parent_counter', 'third_hypo' : 'data_out_foo'})
            smach.Concurrence.add('MID_SM', sm_mid, remapping={'mid_input':'parent_counter', 'my_third' : 'data_out_foo'})

        smach.StateMachine.add('CON', sm_con,
                               transitions={'outcome4':'CON',
                                            'outcome5':'outcome6'})

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()