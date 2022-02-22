#!/usr/bin/env python3

import time
import rospy
import smach
from atemr_msgs.srv import AgentService, AgentServiceResponse, HardwareService, HardwareServiceRequest, WebService, WebServiceResponse, \
                           DBUSService, DBUSServiceRequest
from atemr_msgs.msg import AgentStatus
from app.utils.helper import StateData, sdataDecoder, ErrCodes, ShutdownAction, MapAction, MODE, VIDACTION, AgentStates as astates, AgentKeys as akeys
from app.utils.config import cfgContext
from app.utils.streamer import Streamer
import threading
from multiprocessing import Pipe
from bitarray import bitarray
from bitarray.util import ba2int, int2ba
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Bool, String
from atemr_msgs.msg import Status, WebStatus
from geometry_msgs.msg import PoseWithCovarianceStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus

class MONITORState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success'])
        self.incoming_queue = incoming_queue
        self.outgoing_queue = outgoing_queue

        #rospy.init_node('sm_monitor_node')
        self.agent_srvr = rospy.Service('AgentServer', AgentService, self.agentServe)
        self.webui_srvr = rospy.Service('WebUIServer', WebService, self.webuiServe)
        self.agent_status_pub = rospy.Publisher(cfgContext['agent_topic'], AgentStatus, latch=True, queue_size=1)
        self.atemr_server = actionlib.SimpleActionServer(cfgContext['monitor_server'], MoveBaseAction, execute_cb=self.as_execute_cb, auto_start = False)
        self.agent_status_msg = AgentStatus()
        self.cur_smstate = None

        #wait till node-controller is ready
        nctl_ready = False #SET TO FALSE WHEN RUNNING REAL
        while(not nctl_ready):
            rospy.loginfo('MONITOR - waiting for Node controller ....')
            try:
                msg = rospy.wait_for_message(cfgContext['node_controller_topic'], String, timeout=10)
                if(msg.data == 'READY'):
                    nctl_ready = True
                    break
            except rospy.ROSException as e:
                print(e)
            time.sleep(0.1)

        try:
            rospy.wait_for_service('HARDWAREServer', timeout=5)
            self.hdwClient = rospy.ServiceProxy('HARDWAREServer', HardwareService)
        except rospy.ROSException as e:
            rospy.logerr("Service not avaialable: " + str(e))
            self.errStateTrigger(ErrCodes.SERVICE_NO_EXIST, e)
        
        try:
            rospy.wait_for_service('DBUSServer', timeout=5)
            self.dbusClient = rospy.ServiceProxy('DBUSServer', DBUSService)
        except rospy.ROSException as e:
            rospy.logerr("Service not avaialable: " + str(e))
            self.errStateTrigger(ErrCodes.SERVICE_NO_EXIST, e)
        
        self.module_states = bitarray(8)
        self.module_states.setall(0)
        self.agent_states = bitarray(6)
        self.agent_states.setall(0)
        self._mlock = threading.Lock() #acquisition control for modules
        self._alock = threading.Lock() # acquition control for statemachine
        self.can_run = True
        self.state_updater_thread = threading.Thread(target=self.internalMonitor, daemon=True)
        self.state_updater_thread.start()
        #self.sys_ops_thread = threading.Thread(target=self.systemOps, daemon=True)
        #self.sys_ops_thread.start()

        self.prev_move_status = None
        self.launch_obj = None
        self.parent_conn, self.child_conn = Pipe()
        self.streamer = Streamer(pipe=self.child_conn)

        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cancel_goal = False
        self.is_active = False
        self.feedback = MoveBaseActionFeedback()
        self.result = MoveBaseActionResult()
        self.atemr_server.start() #start action server
    
    def active_cb(self):
        #inform EXC that we are navigating to goal
        self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.EXC, GoalStatus.ACTIVE), astates.EXC))
    
    def feedback_cb(self, msg):
        self.feedback.header.stamp = rospy.Time.now()
        self.feedback.header.frame_id = 'map'
        self.feedback.status = self.mb_client.get_state()
        self.feedback.feedback = msg

    def done_cb(self, state, result):
        self.result.header.stamp = rospy.Time.now()
        self.result.header.frame_id = 'map'
        self.result.status = self.mb_client.get_state()
        self.result.result = result

    def as_execute_cb(self, goal):
        #check if move_base action server is running
        if(self.mb_client.wait_for_server(timeout=rospy.Duration(5.0)) and (self.cur_smstate == astates.IDL)):
            #change to EXC state if current state is IDL
            self.outgoing_queue.put(StateData(akeys.TRIGR_STATE, astates.EXC, astates.IDL))
            #wait till EXC is achieved
            wait_rate = rospy.Rate(2)
            while((self.cur_smstate != astates.EXC)):
                rospy.loginfo('Awaiting EXC state ...')
                wait_rate.sleep()
            
            #reset feedback and result variables
            self.feedback = MoveBaseActionFeedback()
            self.result = MoveBaseActionResult()
            #send goal to move-base server
            goal.header.frame_id = "map"
            self.mb_client.send_goal(goal, active_cb=self.active_cb, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
            #wait for goal to become active
            while(not self.is_active):
                rospy.loginfo('waiting for goal to become active ...')
                if(self.cancel_goal):
                    self.mb_client.cancel_all_goals()
                    break
                wait_rate.sleep()
            
            rate = rospy.Rate(20)
            while(self.is_active and 
                ((self.mb_client.get_state() == GoalStatus.ACTIVE) or (self.mb_client.get_state() == GoalStatus.PENDING ))): #listen to feedback and send back to webserver
                self.atemr_server.publish_feedback(self.feedback)
                if(self.cancel_goal):
                    self.mb_client.cancel_all_goals()
                    break
                if(self.atemr_server.is_preempt_requested()):
                    self.mb_client.cancel_goal()
                    self.atemr_server.set_preempted()
                    new_goal = self.atemr_server.is_new_goal_available()
                    if(new_goal is not None):
                        self.mb_client.send_goal(new_goal, active_cb=self.active_cb, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
                        while(True):
                            rospy.loginfo('waiting for new goal to become active ...')
                            if(self.cancel_goal):
                                self.mb_client.cancel_all_goals()
                                break
                            wait_rate.sleep()
                rate.sleep()
            
            if(self.cancel_goal):
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.EXC, GoalStatus.SUCCEEDED), astates.EXC))
                self.atemr_server.set_preempted()
            if(self.result.status == GoalStatus.SUCCEEDED): #depends on the result
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.EXC, GoalStatus.SUCCEEDED), astates.EXC))
                self.atemr_server.set_succeeded(self.result)
            else:
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.EXC, GoalStatus.ABORTED), astates.EXC))
                self.atemr_server.set_aborted(self.result)
            
            self.is_active = False
            self.cancel_goal = False
            
            

    def checkLocalized(self):
        try:
            msg = rospy.wait_for_message(cfgContext['loc_topic'], PoseWithCovarianceStamped, timeout=2)
            cov = msg.pose.covariance # array of 36
            if((abs(cov[0]) <= 0.08) and (abs(cov[1]) <= 0.01) and (abs(cov[6]) <= 0.01) and (abs(cov[7]) <= 0.05) and (abs(cov[35]) <= 0.012)):
                self._alock.acquire()
                self.agent_states[4] = 1
                self._alock.release()
            else:
                self._alock.acquire()
                self.agent_states[4] = 0
                self._alock.release()
        except rospy.ROSException as e:
            pass

    #determine if connected to WiFi
    def checkWifiConnection(self): 
        try:
            msg = rospy.wait_for_message(cfgContext['dbus_topic'], String, timeout=5)
            self._alock.acquire()
            self.agent_states[3] = 0 if((msg.data == '')) else 1
            #rospy.loginfo(self.agent_states)
            self._alock.release()
        except rospy.ROSException as e:
            rospy.logerr(e)

    
    def internalMonitor(self):
        ''' constantly checks and updates *** agent_states & module_states *** 
                0     |     1      |       2     |          3         |        4       |        5       |
            Heartbeat | Hardware   |Manual/Auto  | Wifi(Connectivity) | Localization   |    Emergency   |

                0     |     1      |  2  |   3   |    4   |     5     |        6       |        7     
            LeftMotor | RightMotor | IMU | Lidar | Camera | WebServer | IRSensor Front | IRSensor Rear
        '''
        while(not rospy.is_shutdown() and self.can_run):
            #time.sleep(3.0) #COMMENT WHEN RUNNING REAL
            #return
            #motor status
            try:
                msg1 = rospy.wait_for_message(cfgContext['base_topic'], Status, timeout=2)
                self._mlock.acquire()
                self.module_states[0] = 1 if((msg1.motorID[0] == 321) and (msg1.state[0] != 0x09)) else 0
                self.module_states[1] = 1 if((msg1.motorID[1] == 322) and (msg1.state[1] != 0x09)) else 0
                self._mlock.release()
                #emergency status
                self._alock.acquire()
                self.agent_states[5] = 1 if((msg1.is_emergency.data == True)) else 0
                self._alock.release()
            except rospy.ROSException as e:
                #print(e)
                pass
            
            #imu status
            try:
                msg1 = rospy.wait_for_message(cfgContext['imu_topic'], Imu, timeout=2)
                self._mlock.acquire()
                self.module_states[2] = 1 if((msg1.header.frame_id == 'wt901_imu')) else 0
                self._mlock.release()
            except rospy.ROSException as e:
                #print(e)
                pass

            #lidar status
            try:
                msg1 = rospy.wait_for_message(cfgContext['rplidar_topic'], LaserScan, timeout=2)
                self._mlock.acquire()
                self.module_states[3] = 1 if((msg1.header.frame_id == 'rplidar')) else 0
                self._mlock.release()
            except rospy.ROSException as e:
                #print(e)
                pass

            #camera status
            try:
                msg1 = rospy.wait_for_message(cfgContext['image_topic'], Image, timeout=5)
                self._mlock.acquire()
                self.module_states[4] = 1 if((msg1.header.frame_id == 'camera_color_optical_frame')) else 0
                self._mlock.release()
            except rospy.ROSException as e:
                #print(e)
                pass
            
            #webserver status
            try:
                msg1 = rospy.wait_for_message(cfgContext['webserver_topic'], WebStatus, timeout=5)
                self._mlock.acquire()
                self.module_states[5] = 1
                self._mlock.release()
            except rospy.ROSException as e:
                self._mlock.acquire()
                self.module_states[5] = 0
                self._mlock.release()
                #print(e)
            
            self._alock.acquire() #upate agent hardware flag
            self.agent_states[1] = 1 if(self.module_states.all()) else 0
            self._alock.release()
            self.checkWifiConnection()
            self.checkLocalized()
            time.sleep(2.0)


    def agentServe(self, req):
        '''queried by the base hardware'''
        # check if robot can shutdown
        if(req.queryShutdown.data):
            resp = AgentServiceResponse()
            if((self.cur_smstate == astates.IDL)):
                dbus_req = DBUSServiceRequest()
                dbus_req.queryPower.data = True
                dbus_resp = self.dbusClient.call(dbus_req)
                if(dbus_resp.canPowerOFF.data):
                    resp.canShutdown.data = True
            else:
                resp.canShutdown.data = False
            return
        
        # initiate transition to SHUTDOWN state on trigger
        if(req.initiateShutdown.data):
            self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.SDWN, (ShutdownAction.SHUTDOWN, self.launch_obj)), astates.IDL))
            return
        
        # set/ unset isAlive status
        # set/ unset frontIRSensor status
        # set/ unset rearIRSensor status
        # set/ unset canMove status
        if(req.is_status_update.data):
            self._mlock.acquire()
            self.module_states[6] = 1 if(req.hasFrontIRSensor.data) else 0
            self.module_states[7] = 1 if(req.hasRearIRSensor.data) else 0
            self._mlock.release()
            self._alock.acquire()
            self.agent_states[0] = 1 if(req.isAlive.data) else 0
            self._alock.release()
            #if(req.canMove.data != self.prev_move_status):
               # self.errStateTrigger(ErrCodes.BASE_EMERGENCY if(not req.canMove.data) else ErrCodes.BASE_OK, '')
                #  self.prev_move_status = req.canMove.data #update move variable
        return AgentServiceResponse()

    def webuiServe(self, req):
        resp = WebServiceResponse()
        resp.canExecute.data = True
        
        if(req.is_operator_action.data and ((self.cur_smstate == astates.IDL) or (self.cur_smstate == astates.ERR))):
            '''state checks are performed in the web UI (browser side)'''
            if(req.initiateShutdown.data): #on_initiateShutdown: trigger transition to SHUTDOWN state (success)
                #print('shutdown request received!')
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.SDWN, (ShutdownAction.SHUTDOWN, self.launch_obj)), self.cur_smstate))
            if(req.initiateRestart.data): #on_initiateRestart: trigger transition to SHUTDOWN state (restart)
                #print('reboot request received!')
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.SDWN, (ShutdownAction.RESTART, self.launch_obj)), self.cur_smstate))
        elif(req.is_operator_action.data and ((self.cur_smstate != astates.IDL) or (self.cur_smstate != astates.ERR))):
                resp.canExecute.data = False
                resp.rejectReason.data = 'Error occurred | Code: ' + ErrCodes.STATE_MISMATCH
                return resp
            
        # toggle man-auto modes
        if(req.is_man_auto.data and ((self.cur_smstate == astates.IDL) or (self.cur_smstate == astates.ERR))):
            if(req.manAuto == MODE.MANUAL.value): #publish keyboard priority
                mpub = rospy.Publisher('/manual_lock', Bool, latch=True, queue_size=1)
                apub = rospy.Publisher('/auto_lock', Bool, latch=True, queue_size=1)
                mpub.publish(False)#deactivate lock
                apub.publish(True) #activate lock
                self._alock.acquire()
                self.agent_states[2] = 0
                self._alock.release()
            if(req.manAuto == MODE.AUTO.value): #publish keyboard priority
                mpub = rospy.Publisher('/manual_lock', Bool, latch=True, queue_size=1)
                apub = rospy.Publisher('/auto_lock', Bool, latch=True, queue_size=1)
                mpub.publish(True)
                apub.publish(False)
                self._alock.acquire()
                self.agent_states[2] = 1
                self._alock.release()
        elif(req.is_man_auto.data and ((self.cur_smstate != astates.IDL) or (self.cur_smstate != astates.ERR))):
                resp.canExecute.data = False
                resp.rejectReason.data = 'Error occurred | Code: ' + ErrCodes.STATE_MISMATCH
                return resp
        
        # image capture / video stream commands
        if(req.is_stream.data):
            # capture image
            self.parent_conn.send(StateData(akeys.IMG_STRM, req.captureImage.data))
            # start/save video capture
            self.parent_conn.send(StateData(akeys.VID_STRM, req.captureVideo))
        
        # trigger map_actions
        if((req.is_map_action.data) and (self.cur_smstate == astates.IDL)): 
            self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.MAP, (MapAction(req.mapAction), req.mapName.data)), astates.IDL))
        elif((req.is_map_action.data) and (self.cur_smstate == astates.MAP)): 
            self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.IDL, (MapAction(req.mapAction), req.mapName.data)), astates.MAP))
        elif((req.is_map_action.data) and ((self.cur_smstate != astates.MAP) or (self.cur_smstate != astates.IDL))):
            resp.canExecute.data = False
            resp.rejectReason.data = 'Error occurred | Code: ' + ErrCodes.STATE_MISMATCH
            return resp
        
        # trigger goal execution
        if((req.is_goal_action.data) and (self.cur_smstate == astates.IDL)):
            self.outgoing_queue.put(StateData(akeys.TRIGR_STATE, astates.EXC, astates.IDL))
        elif((req.is_goal_action.data) and (self.cur_smstate == astates.EXC)):
            self.cancel_goal = req.cancel_goal.data
            #self.outgoing_queue.put(StateData(akeys.TRIGR_STATE_EXTRA, (astates.IDL, req.cancel_goal.data), astates.EXC))
        elif((req.is_goal_action.data) and ((self.cur_smstate != astates.IDL) or (self.cur_smstate != astates.EXC))):
            resp.canExecute.data = False
            resp.rejectReason.data = 'Error occurred | Code: ' + ErrCodes.STATE_MISMATCH
            return resp
        
        return resp

    def errStateTrigger(self, code, err):
        if(self.cur_smstate == astates.IDL):
            if(code == ErrCodes.BASE_EMERGENCY):
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE, astates.ERR, astates.IDL))
                self.outgoing_queue.put(StateData(akeys.ERR_OBJ, (code, 'Base blocked!'), astates.ERR))
            if(code == ErrCodes.SERVICE_NO_EXIST):
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE, astates.ERR, astates.IDL))
                self.outgoing_queue.put(StateData(akeys.ERR_OBJ, (code, err), astates.ERR))
        
        elif(self.cur_smstate == astates.ERR):
            if(code == ErrCodes.BASE_EMERGENCY):
                self.outgoing_queue.put(StateData(akeys.ERR_OBJ, (code, 'Base blocked!'), astates.ERR))
            if(code == ErrCodes.BASE_OK):
                self.outgoing_queue.put(StateData(akeys.TRIGR_STATE, astates.IDL, astates.ERR))
        
        else: #any other state
            rospy.logerr('ERROR -> %d | %s' % (code.value, str(err)))        


    def execute(self, userdata):
        rospy.loginfo('Monitoring ...')
        self.streamer.start()
        rate = rospy.Rate(20)
        pwr_on_ACK = False

        while(not rospy.is_shutdown()):
            rospy.loginfo_throttle(3, "MONITOR running ....")

            #check and process incoming data
            if(not self.incoming_queue.empty()):
                data_obj = sdataDecoder(self.incoming_queue, astates.MON)
                if(data_obj is not None): #process received queue data here
                    if((data_obj.name == akeys.SM_STATE)):
                        #rospy.loginfo(data_obj.dataObject)
                        self.agent_status_msg.agentSMState.data = data_obj.dataObject.value
                        self.cur_smstate = data_obj.dataObject
                        if(self.cur_smstate == astates.SDWN):
                            rospy.loginfo('System going into shutdown (reboot)')
                            break
                    #+++++++++++Acknowledge POWER ON on request from IDLE state
                    if((data_obj.name == akeys.TRIGR_ACK) and (data_obj.dataObject) and (not pwr_on_ACK)):
                        try:
                            req = HardwareServiceRequest()
                            req.powerON_ACK.data = True
                            if(self.hdwClient(req)):
                                pwr_on_ACK = True
                        except rospy.ServiceException as exc:
                            print("Service did not process request: " + str(exc))
                            # trigger transition to ERROR State
                    
                    #+++++++++++Trigger motor reset on request from ERROR state
                    if((data_obj.name == akeys.TRIGR_M_RESET) and (data_obj.dataObject)):
                        try:
                            req = HardwareServiceRequest()
                            req.resetMotors.data = True
                            self.hdwClient(req)
                        except rospy.ServiceException as exc:
                            print("Service did not process request: " + str(exc))
                            # trigger transition to ERROR State
                    
                    #+++++++++++Receive Objects
                    if((data_obj.name == akeys.LNCH_OBJ)):
                        self.launch_obj = data_obj.dataObject
                    if((data_obj.name == akeys.MOD_STATES)):
                        self._mlock.acquire()
                        self.module_states[1:6] = data_obj.dataObject[0:5]
                        self._mlock.release()
            
            #update module states and publish
            self.agent_status_msg.agentStatus = ba2int(self.agent_states)
            self.agent_status_msg.hardwareStatus = ba2int(self.module_states)
            self.agent_status_pub.publish(self.agent_status_msg)
            rate.sleep()
        
        self.can_run = False
        return 'success'