#!/usr/bin/env python3
from bitarray import bitarray
import roslaunch
import rospy
from app.utils.config import cfgContext, rospack
from rospy.exceptions import ROSException
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Image
from nav_msgs.msg import Odometry
from atemr_msgs.msg import Status
import time
from enum import Enum
from bitarray.util import ba2int

class NodeType(Enum):
    BASE = 0
    IMU = 1
    LIDAR = 2
    CAMERA = 3
    RLOC_ODOM = 4
    RLOC_WORLD = 5

class RobotLauncher:
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

    def launch_base(self, terminate=False):
        if(not terminate):
            self.base_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [rospack.get_path('atemr_hardware') + '/launch/base.launch'])
            self.base_launch.start()
        else:
            self.base_launch.shutdown()
        time.sleep(3.0)
    
    def launch_imu(self, terminate=False):
        if(not terminate):
            self.imu_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [rospack.get_path('atemr_hardware') + '/launch/imu.launch'])
            self.imu_launch.start()
        else:
            self.imu_launch.shutdown()
        time.sleep(1.0)
    
    def launch_lidar(self, terminate=False):
        if(not terminate):
            self.lidar_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [rospack.get_path('atemr_hardware') + '/launch/lidar.launch'])
            self.lidar_launch.start()
        else:
            self.lidar_launch.shutdown()
        time.sleep(1.0)
    
    def launch_camera(self, terminate=False):
        if(not terminate):
            self.camera_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [rospack.get_path('atemr_hardware') + '/launch/camera.launch'])
            self.camera_launch.start()
        else:
            self.camera_launch.shutdown()
        time.sleep(1.5)
    
    def launch_rloc_odom(self, terminate=False):
        if(not terminate):
            self.rloc_odom_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [rospack.get_path('atemr_localization') + '/launch/rloc_odom.launch'])
            self.rloc_odom_launch.start()
        else:
            self.rloc_odom_launch.shutdown()
        time.sleep(3.0)
    
    def launch_rloc_world(self, terminate=False):
        if(not terminate):
            self.rloc_world_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [rospack.get_path('atemr_localization') + '/launch/rloc_world.launch'])
            self.rloc_world_launch.start()
        else:
            self.rloc_world_launch.shutdown()
        time.sleep(2.0)


    def run(self, module_states):
        #begin checks
        #check BASE
        if(module_states[0] == 0):
            self.launch_base()
            try:
                msg1 = rospy.wait_for_message(cfgContext['base_topic'], Status, timeout=5)
                if((msg1.motorID[0] == 0x141) and (msg1.motorID[1] == 0x142) and (msg1.state[0] != 0x09) and (msg1.state[1] != 0x09)):
                    module_states[0] = 1
                else:
                    print('BASE validation failed!')
            except ROSException as e:
                print(e)
                module_states[0] = 0
        
        #check IMU
        if(module_states[1] == 0):
            self.launch_imu()
            try:
                msg1 = rospy.wait_for_message(cfgContext['imu_topic'], Imu, timeout=5)
                #msg2 = rospy.wait_for_message(cfgContext['imu_filter_topic'], Imu, timeout=5)
                if((msg1.header.frame_id == 'wt901_imu')): # or (msg2.header.frame_id == 'wt901_imu')):
                    module_states[1] = 1
                else:
                    print('IMU node validation failed!')
            except rospy.ROSException as e:
                print(e)
                module_states[1] = 0
        
        #check LIDAR
        if(module_states[2] == 0):
            self.launch_lidar()
            try:
                msg1 = rospy.wait_for_message(cfgContext['rplidar_topic'], LaserScan, timeout=5)
                if((msg1.header.frame_id == 'rplidar')):
                    module_states[2] = 1
                else:
                    print('LIDAR node validation failed!')
            except rospy.ROSException as e:
                print(e)
                module_states[2] = 0
        
        #check CAMERA
        if(module_states[3] == 0):
            self.launch_camera()
            try:
                msg1 = rospy.wait_for_message(cfgContext['depth_topic'], PointCloud2, timeout=5)
                msg2 = rospy.wait_for_message(cfgContext['image_topic'], Image, timeout=5)
                if((msg1.header.frame_id == 'camera_depth_optical_frame') and (msg2.header.frame_id == 'camera_color_optical_frame')):
                    module_states[3] = 1
                else:
                    print('CAMERA node validation failed!')
            except rospy.ROSException as e:
                print(e)
                module_states[3] = 0
        
        #check RLOC ODOM
        if(module_states[4] == 0):
            self.launch_rloc_odom()
            try:
                msg1 = rospy.wait_for_message(cfgContext['rloc_odom_topic'], Odometry, timeout=5)
                if((msg1.header.frame_id == 'odom')):
                    module_states[4] = 1
                else:
                    print('RLOC ODOM node validation failed!')
            except rospy.ROSException as e:
                print(e)
                module_states[4] = 0
        
        return module_states

    
    def terminate(self, module_states=bitarray(5), isRetry=False):
        if(isRetry):
            if(module_states[0] == 0):
                try:
                    self.launch_base(terminate=True)
                    time.sleep(5.0)
                except ROSException as e:
                    print(e)
            if(module_states[1] == 0):
                try:
                    self.launch_imu(terminate=True)
                except ROSException as e:
                    print(e)
            if(module_states[2] == 0):
                try:
                    self.launch_lidar(terminate=True)
                except ROSException as e:
                    print(e)
            if(module_states[3] == 0):
                try:
                    self.launch_camera(terminate=True)
                except ROSException as e:
                    print(e)
            
            try:# always shutdown [4]
                self.launch_rloc_world(terminate=True)
            except ROSException as e:
                print(e)
            module_states[4] == 0
        else: # complete shutdown
            if(ba2int(module_states) == 0):
                return module_states

            try:
                self.launch_rloc_world(terminate=True)
            except ROSException as e:
                print(e)
            try:
                self.launch_rloc_odom(terminate=True)
            except ROSException as e:
                print(e)
            try:
               self.launch_imu(terminate=True)
            except ROSException as e:
                print(e)
            try:
                self.launch_lidar(terminate=True)
            except ROSException as e:
                print(e)
            try:
                self.launch_camera(terminate=True)
            except ROSException as e:
                print(e)
            #try:
            #    self.launch_base(terminate=True)
            #    time.sleep(5.0)
            #except ROSException as e:
            #    print(e)
            module_states.setall(0)
            module_states[NodeType.BASE.value] = 1
        time.sleep(3.0)
        return module_states


if __name__ == '__main__':
    rospy.init_node('launcher_test_node', anonymous=True)
    launcher = RobotLauncher()
    m_states = bitarray(5)
    m_states.setall(0)
    m_states = launcher.run(module_states=m_states)
    time.sleep(10)
    print("Launch States: ", m_states)
    launcher.terminate()
    exit()


