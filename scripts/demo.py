#! /usr/bin/env python3

import threading
import time
import rospy
from rospy.service import ServiceException
from atemr_msgs.srv import DBUSService, DBUSServiceRequest, DBUSServiceResponse
from std_msgs.msg import Bool

class ThreadTest:
    def __init__(self):
        self.module_states = 0
        self.thrd = threading.Thread(target=self.internalMonitor, daemon=True)
        self.thrd.start()
    
    def internalMonitor(self):
        ''' constantly checks and updates ***module_states & agent_states*** '''
        while(True):
            self.module_states += 1
            time.sleep(1.0)
            if(self.module_states > 8):
                break
    
    def run(self):
        while(True):
            print('Value is %d' % self.module_states)
            time.sleep(0.5)
            if(self.module_states > 5):
                break

if __name__ == '__main__':
    '''
    try:
        rospy.wait_for_service('DBUSServer', timeout=5)
        server = rospy.ServiceProxy('DBUSServer', DBUSService)
    except rospy.ROSException as e:
        print("Service not avaialable: " + str(e))

    req = DBUSServiceRequest()
    req.queryPower.data = True
    if(server.call(req)):
        print('Success')
    
    thrd = ThreadTest()
    thrd.run()
    '''
    rospy.init_node('demoNode', anonymous=True)
    mpub = rospy.Publisher('/manual_lock', Bool, latch=True, queue_size=1)
    apub = rospy.Publisher('/auto_lock', Bool, latch=True, queue_size=1)
    mpub.publish(True)#deactivate lock
    apub.publish(False) #activate lock
    rospy.spin()