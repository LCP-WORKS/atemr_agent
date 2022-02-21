#!/usr/bin/env python3

import rospy, rospkg
import smach
import os
from app.utils.helper import StateData, sdataDecoder, AgentKeys as akeys,\
                            AgentStates as astates, ErrCodes, MapAction
from nav_msgs.srv import LoadMap
from atemr_msgs.srv import NodeControllerService, NodeControllerServiceRequest
from bitarray.util import int2ba
from app.utils.robot_launcher import NodeType as NT
import subprocess


class MAPState(smach.State):
    def __init__(self, incoming_queue, outgoing_queue):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['map_data_i'], output_keys=['err_obj_o'])
        self.in_queue = incoming_queue
        self.out_queue = outgoing_queue
        
        rospack = rospkg.RosPack()
        self.mapPath = rospack.get_path('atemr_localization') + '/maps'
        self.node_ctlClient = rospy.ServiceProxy('NodeControllerServer', NodeControllerService)

    def change_map(self, map_name):
        try:
            rospy.wait_for_service('NodeControllerServer', timeout=5)
            req = NodeControllerServiceRequest()
            resp = self.node_ctlClient.call(req)
            node_states = int2ba(resp.hardwareStatus)
            #check if map server node is running else start it
            if(node_states[NT.MAPSERVER.value] == 0):
                req = NodeControllerServiceRequest()
                req.is_mapserver.data = True
                req.mapserver_action.data = True
                resp = self.node_ctlClient.call(req)
                node_states = int2ba(resp.hardwareStatus)
                rospy.loginfo('Attempt to START map server status: %i' % node_states[NT.MAPSERVER.value])

            #check if rloc_world is running else start it
            if(node_states[NT.RLOC_WORLD.value] == 0):
                req = NodeControllerServiceRequest()
                req.is_rloc_world.data = True
                req.rloc_world_action.data = True
                resp = self.node_ctlClient.call(req)
                node_states = int2ba(resp.hardwareStatus)
                rospy.loginfo('Attempt to START robot localization world status: %i' % node_states[NT.RLOC_WORLD.value])
                
            rospy.wait_for_service('change_map', timeout=5)
            try:
                change_map_proxy = rospy.ServiceProxy('change_map', LoadMap)
                map_url = os.path.join(self.mapPath, map_name, map_name + '.yaml')
                resp = change_map_proxy(map_url)
                return True #if(resp.result == 0) else False
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr(e)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)
        return False
    
    def start_stop_mapping(self, stop = False):
        #send request to start mapping node and kill the map-server node
        try:
            rospy.wait_for_service('NodeControllerServer', timeout=5)
            req = NodeControllerServiceRequest()
            resp = self.node_ctlClient.call(req)
            node_states = int2ba(resp.hardwareStatus)
            if(not stop): #begin mapping
                #check if map server node is running then stop it
                if(node_states[NT.MAPSERVER.value] == 1):
                    req = NodeControllerServiceRequest()
                    req.is_mapserver.data = True
                    req.mapserver_action.data = False
                    resp = self.node_ctlClient.call(req)
                    node_states = int2ba(resp.hardwareStatus)
                    rospy.loginfo('Attempt to STOP map server status: %i' % node_states[NT.MAPSERVER.value])

                #check if rloc_world is running then stop it
                if(node_states[NT.RLOC_WORLD.value] == 1):
                    req = NodeControllerServiceRequest()
                    req.is_rloc_world.data = True
                    req.rloc_world_action.data = False
                    resp = self.node_ctlClient.call(req)
                    node_states = int2ba(resp.hardwareStatus)
                    rospy.loginfo('Attempt to STOP robot localization world status: %i' % node_states[NT.RLOC_WORLD.value])
                
                #start the mapping node
                req = NodeControllerServiceRequest()
                req.is_mapping.data = True
                req.mapping_action.data = True
                resp = self.node_ctlClient.call(req)
                node_states = int2ba(resp.hardwareStatus)
                rospy.loginfo('Attempt to START mapping status: %i' % node_states[NT.MAPPING.value])

            else: #stop mapping
                #stop the mapping node
                req = NodeControllerServiceRequest()
                req.is_mapping.data = True
                req.mapping_action.data = False
                resp = self.node_ctlClient.call(req)
                node_states = int2ba(resp.hardwareStatus)
                rospy.loginfo('Attempt to STOP mapping status: %i' % node_states[NT.MAPPING.value])
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(e)

        return True
    
    def save_map(self, map_name):
        
        if(not os.path.exists(self.mapPath)):
            os.mkdir(self.mapPath)
        map_dir = os.path.join(self.mapPath, map_name)
        if(not os.path.exists(map_dir)):
            os.mkdir(map_dir) #create new map directory
        os.chdir(map_dir) #cd to the map directory
        
        # save new map
        result = os.popen('rosrun map_server map_saver -f %s' % map_name)
        if('Done' in result.read()):
            result.close()
            return True
        return False

    def execute(self, userdata):
        rospy.loginfo('Mapping ...')
        rate = rospy.Rate(1)
        outcome = None
        is_making_map = False

        self.out_queue.put(StateData(akeys.SM_STATE, astates.MAP))
        while(not rospy.is_shutdown()):
            rospy.loginfo("MAP running ....")

            if(not self.in_queue.empty()):
                msg_obj = sdataDecoder(self.in_queue, astates.MAP)
                if(msg_obj is not None): #process received queue data here
                    if(msg_obj.name == akeys.TRIGR_STATE_EXTRA):
                        (sm_state, data) = msg_obj.dataObject
                        #process goal commands
                        if((sm_state == astates.IDL) and (data[0] == MapAction.SAVE_NEW_MAP.value) and is_making_map):
                            is_making_map = False
                            #save map under the provided name
                            self.save_map(data[1])
                            #kill mapping node and set current map to the new map
                            outcome = 'success' if(self.start_stop_mapping(stop=True) and self.change_map(data[1])) else 'failure'
                        if((sm_state == astates.IDL) and (data[0] == MapAction.DISCARD_NEW_MAP.value) and is_making_map):
                            is_making_map = False
                            outcome = 'success' if(self.start_stop_mapping(stop=True)) else 'failure'
                    else:
                        rospy.logwarn('Received unknown from queue: %s' % msg_obj.name)
                    msg_obj = None

            if(userdata.map_data_i[0].value == MapAction.CHANGE_MAP.value):
                res = self.change_map(userdata.map_data_i[1])
                if(res):
                    outcome = 'success'
                    break
                else:
                    outcome = 'failure'
                    break
            if(userdata.map_data_i[0].value == MapAction.BEGIN_NEW_MAP.value): #start the mapping node
                if(not is_making_map):
                    if(self.start_stop_mapping()):
                        is_making_map = True
                    else:
                        outcome = 'failure'
                        break
                #userdata.map_data_i = None
            
            if(is_making_map):
                rospy.loginfo("Is Mapping ....")
            else:
                outcome = 'failure'
                break
            rate.sleep()

        return outcome