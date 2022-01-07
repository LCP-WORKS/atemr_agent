import smach
import rospy

# define state Zas
class Zas(smach.State):
    def __init__(self, state_queue):
        smach.State.__init__(self, outcomes=['outcome6'], input_keys=['third_hypo_zas'])
        self.state_queue = state_queue

    def execute(self, userdata):
        rospy.loginfo('Executing state ZAS')
        rospy.loginfo(userdata.third_hypo_zas)
        rate = rospy.Rate(1)
        self.state_queue.clear()
        while(not rospy.is_shutdown()):
            if(not self.state_queue.empty()):
                msg = self.state_queue.get(False)
                print('Received: %i' % msg)
                if(msg > 9):
                    break
        return 'outcome6'