#! /usr/bin/env python3

import rospy
import random
from bitarray import bitarray
from bitarray.util import ba2int
from std_msgs.msg import Bool
from atemr_msgs.msg import Status, AgentStatus, WebStatus, Feedback
from atemr_msgs.srv import WebService, WebServiceResponse
from sensor_msgs.msg import Image
from app.utils.helper import VIDACTION, MODE, StateData, AgentKeys as akeys
from app.utils.config import cfgContext, rospack
from app.utils.streamer import Streamer
from multiprocessing import Pipe


class ATEMRLab:
    def __init__(self):
        ''' constantly checks and updates ***module_states & agent_states*** 
                0     |     1      |       2     |          3         |        4       |
            Heartbeat | Hardware   |Manual/Auto  | Wifi(Connectivity) | Localization   |

                0     |     1      |  2  |   3   |    4   |     5     |        6       |        7     
            LeftMotor | RightMotor | IMU | Lidar | Camera | WebServer | IRSensor Front | IRSensor Rear
        '''
        self.agent_states = bitarray(5)
        self.agent_states.setall(1)
        self.agent_states[2] = 0
        self.module_states = bitarray(2 ** 3)
        self.module_states.setall(1)
        #self.module_states[0] = 0
        #self.module_states[7] = 1

        self.webui_srvr = rospy.Service('WebUIServer', WebService, self.webuiServe)
        self.agent_status_pub = rospy.Publisher(cfgContext['agent_topic'], AgentStatus, latch=True, queue_size=1)
        self.agent_status_msg = AgentStatus()
        self.wbsrvr = rospy.Subscriber(cfgContext['webserver_topic'], WebStatus, self.webserverCB)
        self.record_in_session = False
        self.feedback_pub = rospy.Publisher('base_controller/feedback', Feedback, latch=True, queue_size=1)
        self.feedback_msg = Feedback()

        self.parent_conn, self.child_conn = Pipe()
        self.strmr = Streamer(pipe=self.child_conn)
    
    def webserverCB(self, msg):
        self.module_states[5] = 1 if((msg.isAlive.data == True)) else 0

    def webuiServe(self, req):
        resp = WebServiceResponse()
        resp.canExecute.data = True
        if(req.is_operator_action.data):
            '''state checks are performed in the web UI (browser side)'''
            if(req.initiateShutdown.data): #on_initiateShutdown: trigger transition to SHUTDOWN state (success)
                rospy.loginfo('SHUTDOWN requested!')
            if(req.initiateRestart.data): #on_initiateRestart: trigger transition to SHUTDOWN state (restart)
                rospy.loginfo('RESTART requested!')
            
        # toggle man-auto modes
        if(req.is_man_auto.data):
            if(req.manAuto == MODE.MANUAL.value): #publish keyboard priority
                mpub = rospy.Publisher('/manual_lock', Bool, latch=True, queue_size=1)
                apub = rospy.Publisher('/auto_lock', Bool, latch=True, queue_size=1)
                mpub.publish(False)#deactivate lock
                apub.publish(True) #activate lock
                self.agent_states[2] = 0
                rospy.loginfo('MANUAL mode triggered!')
            if(req.manAuto == MODE.AUTO.value): #publish keyboard priority
                mpub = rospy.Publisher('/manual_lock', Bool, latch=True, queue_size=1)
                apub = rospy.Publisher('/auto_lock', Bool, latch=True, queue_size=1)
                mpub.publish(True)
                apub.publish(False)
                self.agent_states[2] = 1
                rospy.loginfo('AUTO mode triggered!')
        
        if(req.is_stream.data):
            # REAL CODE
            # capture image
            self.parent_conn.send(StateData(akeys.IMG_STRM, req.captureImage.data))
            # start/save video capture
            self.parent_conn.send(StateData(akeys.VID_STRM, req.captureVideo))

            # DEMO CODE
            # capture image
            if(req.captureImage.data):
                rospy.loginfo('CAPTURED Image !!')
            # start/save video capture
            if(req.captureVideo == VIDACTION.START.value):
                if(not self.record_in_session):
                    rospy.loginfo('RECORDING in session!')
                    self.record_in_session = True
            if(req.captureVideo == VIDACTION.SAVE.value):
                if(self.record_in_session):
                    rospy.loginfo('RECORDING saved!')
                    self.record_in_session = False
                else:
                    rospy.loginfo('not RECORDING!')
            if(req.captureVideo == VIDACTION.CANCEL.value):
                if(self.record_in_session):
                    rospy.loginfo('RECORDING not saved!')
                    self.record_in_session = False
                else:
                    rospy.loginfo('not RECORDING!')
        
        # trigger map_actions
        if(req.is_map_action.data): 
            rospy.loginfo('Received MAP request: %d - name: %s' % (req.mapAction, req.mapName.data))
        # trigger goal execution
        if(req.is_goal_action.data):
            rospy.loginfo('Received NAV request: ')
            rospy.loginfo(req.goal)
            rospy.loginfo('+++++++++++++++++++++++')
        return resp

    def session(self):
        rospy.init_node('atemr_lab_NODE', anonymous=False)
        rospy.loginfo('Experimenting ...')
        self.strmr.start()
        img_toggle = False
        rate = rospy.Rate(15)
        while(not rospy.is_shutdown()):
            self.agent_status_msg.agentSMState.data = 'IDLE'
            self.agent_status_msg.agentStatus = ba2int(self.agent_states)
            self.agent_status_msg.hardwareStatus = ba2int(self.module_states)
            #update module states and publish
            self.agent_status_pub.publish(self.agent_status_msg)
            #publish randomized feedback
            self.feedback_msg.meas_vel[0] = random.randrange(-10, 10) #publish rad/s
            self.feedback_msg.meas_vel[1] = random.randrange(-10, 10)
            self.feedback_pub.publish(self.feedback_msg)
            #publish demo camera image
            self.image_file = rospack.get_path('atemr_agent') + '/data/images/demo1.png' if(img_toggle) else rospack.get_path('atemr_agent') + '/data/images/demo2.png'
            self.strmr.publishOnce(self.image_file)
            img_toggle = not img_toggle
            try:
                rate.sleep()
            except (rospy.exceptions.ROSTimeMovedBackwardsException, rospy.exceptions.ROSTimeMovedBackwardsException) as ex:
                pass


if __name__ == '__main__':
    lab = ATEMRLab()
    lab.session()
    exit(0)

