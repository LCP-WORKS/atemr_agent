#!/usr/bin/env python
from bitarray import bitarray
import roslaunch
import rospy
from app.utils.config import cfgContext, rospack
from rospy.exceptions import ROSException
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Image
from atemr_msgs.msg import Status
import time

class RobotLauncher:
    def __init__(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.base_launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('atemr_hardware') + '/launch/base.launch'])
        self.imu_launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('atemr_hardware') + '/launch/imu.launch'])
        self.lidar_launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('atemr_hardware') + '/launch/lidar.launch'])
        self.camera_launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('atemr_hardware') + '/launch/camera.launch'])

    def run(self, module_states):
        #begin checks
        #check BASE
        if(module_states[0] == 0):
            self.base_launch.start()
            time.sleep(3.0)
            try:
                msg1 = rospy.wait_for_message(cfgContext['base_topic'], Status, timeout=5)
                if((msg1.motorID[0] == 0x141) and (msg1.motorID[1] == 0x142) and (msg1.state[0] != 0x09) and (msg1.state[1] != 0x09)):
                    module_states[0] = 1
                else:
                    print('BASE validation failed!')
            except rospy.ROSException as e:
                print(e)
                module_states[0] = 0
        
        #check IMU
        if(module_states[1] == 0):
            self.imu_launch.start()
            time.sleep(1.5)
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
            self.lidar_launch.start()
            time.sleep(1.5)
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
            self.camera_launch.start()
            time.sleep(1.5)
            try:
                msg1 = rospy.wait_for_message(cfgContext['camera_topic'], PointCloud2, timeout=5)
                msg2 = rospy.wait_for_message(cfgContext['image_topic'], Image, timeout=5)
                #msg2 = rospy.wait_for_message(cfgContext['camera_laser_filter'], LaserScan, timeout=5)
                if((msg1.header.frame_id == 'realsense_camera') and (msg2.header.frame_id == 'realsense_camera')):
                    module_states[3] = 1
                else:
                    print('CAMERA node validation failed!')
            except rospy.ROSException as e:
                print(e)
                module_states[3] = 0

        return module_states

    
    def terminate(self, module_states=bitarray(4), isRetry=False):
        if(isRetry):
            if(module_states[0] == 0):
                self.base_launch.shutdown()
            if(module_states[1] == 0):
                self.imu_launch.shutdown()
            if(module_states[2] == 0):
                self.lidar_launch.shutdown()
            if(module_states[3] == 0):
                self.camera_launch.shutdown()
        else:
            self.imu_launch.shutdown()
            self.lidar_launch.shutdown()
            self.camera_launch.shutdown()
            self.base_launch.shutdown()
        time.sleep(3.0)


if __name__ == '__main__':
    launcher = RobotLauncher()
    m_states = bitarray(4)
    m_states.setall(0)
    m_states = launcher.run(module_states=m_states)
    time.sleep(10)
    print(m_states)
    launcher.terminate()
    exit()


