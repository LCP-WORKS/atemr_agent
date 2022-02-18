#!/usr/bin/env python3

from glob import glob
from platform import node
import rospy
from std_msgs.msg import String
from atemr_msgs.srv import NodeControllerService, NodeControllerServiceResponse
from bitarray import bitarray
from bitarray.util import ba2int
from app.utils.config import cfgContext
from app.utils.robot_launcher import RobotLauncher, NodeType
import time

nodes_launcher = RobotLauncher()
node_states = bitarray(7)
init = False

def request_handler(req):
    global nodes_launcher, init, node_states
    if(req.is_basics.data):
        if(req.basics_action.data):
            if(init): #then restart
                node_states = nodes_launcher.terminate(module_states=node_states, isRetry=False)
                node_states = nodes_launcher.run(module_states=node_states)
            else:
                node_states = nodes_launcher.run(module_states=node_states)
        else:
            node_states = nodes_launcher.terminate(module_states=node_states, isRetry=False)
    
    if(req.is_base.data):
        if(req.base_action.data): #start/restart
            if(node_states[NodeType.BASE.value] == 1):
                nodes_launcher.launch_base(True)
            nodes_launcher.launch_base(False)
            node_states[NodeType.BASE.value] = 1
        else:
            nodes_launcher.launch_base(True)
            node_states[NodeType.BASE.value] = 0
    
    if(req.is_imu.data):
        if(req.imu_action.data): #start/restart
            if(node_states[NodeType.IMU.value] == 1):
                nodes_launcher.launch_imu(True)
            nodes_launcher.launch_imu(False)
            node_states[NodeType.IMU.value] = 1
        else:
            nodes_launcher.launch_imu(True)
            node_states[NodeType.IMU.value] = 0
    
    if(req.is_lidar.data):
        if(req.lidar_action.data): #start/restart
            if(node_states[NodeType.LIDAR.value] == 1):
                nodes_launcher.launch_lidar(True)
            nodes_launcher.launch_lidar(False)
            node_states[NodeType.LIDAR.value] = 1
        else:
            nodes_launcher.launch_lidar(True)
            node_states[NodeType.LIDAR.value] = 0
    
    if(req.is_camera.data):
        if(req.camera_action.data): #start/restart
            if(node_states[NodeType.CAMERA.value] == 1):
                nodes_launcher.launch_camera(True)
            nodes_launcher.launch_camera(False)
            node_states[NodeType.CAMERA.value] = 1
        else:
            nodes_launcher.launch_camera(True)
            node_states[NodeType.CAMERA.value] = 0
    
    if(req.is_rloc_odom.data):
        if(req.rloc_odom_action.data): #start/restart
            if(node_states[NodeType.RLOC_ODOM.value] == 1):
                nodes_launcher.launch_rloc_odom(True)
            nodes_launcher.launch_rloc_odom(False)
            node_states[NodeType.RLOC_ODOM.value] = 1
        else:
            nodes_launcher.launch_rloc_odom(True)
            node_states[NodeType.RLOC_ODOM.value] = 0
    
    if(req.is_rloc_world.data):
        if(req.rloc_world_action.data): #start/restart
            if(node_states[NodeType.RLOC_WORLD.value] == 1):
                nodes_launcher.launch_rloc_world(True)
            nodes_launcher.launch_rloc_world(False)
            node_states[NodeType.RLOC_WORLD.value] = 1
        else:
            nodes_launcher.launch_rloc_world(True)
            node_states[NodeType.RLOC_WORLD.value] = 0
    
    if(req.is_mapserver.data):
        if(req.mapserver_action.data): #start/restart
            if(node_states[NodeType.MAPSERVER.value] == 1):
                nodes_launcher.launch_mapserver(True)
            nodes_launcher.launch_mapserver(False)
            node_states[NodeType.MAPSERVER.value] = 1
        else:
            nodes_launcher.launch_mapserver(True)
            node_states[NodeType.MAPSERVER.value] = 0

    if(req.is_mapping.data):
        if(req.mapping_action.data): #start/restart
            if(node_states[NodeType.MAPPING.value] == 1):
                nodes_launcher.launch_mapping(True)
            nodes_launcher.launch_mapping(False)
            node_states[NodeType.MAPPING.value] = 1
        else:
            nodes_launcher.launch_mapping(True)
            node_states[NodeType.MAPPING.value] = 0
    
    resp = NodeControllerServiceResponse()
    resp.result.data = True
    resp.hardwareStatus = ba2int(node_states[:5])
    return resp


if __name__ == '__main__':
    rospy.init_node('node_controller')
    node_states.setall(0)

    status_pub = rospy.Publisher(cfgContext['node_controller_topic'], String, latch=True, queue_size=1)
    srvr = rospy.Service('NodeControllerServer', NodeControllerService, request_handler)

    r = rospy.Rate(10)
    
    while(not rospy.is_shutdown()):
        rospy.loginfo_throttle(5, "Node controller running ....")
        if(not init):
            node_states = nodes_launcher.run(module_states=node_states)
            msg = String()
            msg.data = 'READY'
            status_pub.publish(msg)
            init = True
        r.sleep
    
    rospy.loginfo("Terminating session ...")
    nodes_launcher.terminate(module_states=node_states, isRetry=False)
