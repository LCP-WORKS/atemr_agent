#! /usr/bin/env python3

import rospy
from datetime import datetime
from rospy.exceptions import ROSException
from sensor_msgs.msg import Image
from app.utils.config import cfgContext, rospack
from app.utils.helper import AgentKeys as akeys, VIDACTION
from multiprocessing import Process
import threading
import cv2, os
from cv_bridge import CvBridge, CvBridgeError

class Streamer:
    def __init__(self, pipe):
        self.pipe = pipe
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.imgDest = rospack.get_path('atemr_agent') + '/data/'
        self.vaction = None
        self.srec = False
        self.proc = threading.Thread(target=self.run, args=(self.pipe,), daemon=True)
    
    def start(self):
        self.proc.start()

    def publishOnce(self, img):
        imgPub = rospy.Publisher('rs_cam/colour/image_raw', Image, queue_size=1, latch=True)
        try:
            try:
                cv_image = cv2.imread(img)
                msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            except (CvBridgeError, TypeError) as err:
                print(err)
            msg.header.frame_id = 'realsense_camera'
            msg.header.stamp = rospy.Time.now()
            imgPub.publish(msg)
        except rospy.ROSException as e:
            print(e)

    def captureImage(self):
        try:
            imgMsg = rospy.wait_for_message(cfgContext['image_topic'], Image, timeout=2)
            try:
                cv_image = self.bridge.imgmsg_to_cv2(imgMsg, "bgr8")
                today = datetime.now()
                filename = self.imgDest + ('images/%d-%d-%d_%d:%d:%d.jpeg' %(today.day, today.month, today.year, today.hour, today.minute, today.second))
                cv2.imwrite(filename, cv_image)
            except (CvBridgeError, TypeError) as ce:
                print(str(ce))
        except rospy.ROSException as e:
            print(str(e))

    def captureVideo(self, msg):
        #rospy.loginfo_throttle(1, self.srec)
        if(self.srec):
            rospy.loginfo_throttle(3, 'STRMR: recording')
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.vwriter.write(cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA))
            except CvBridgeError as ce:
                print(ce)
        
        if(self.srec and (self.vaction == VIDACTION.SAVE.value)):
            rospy.loginfo('STRMR: saving')
            self.vwriter.release()
            self.srec = False
            self.vaction = None
        if(self.srec and (self.vaction == VIDACTION.CANCEL.value)):
            rospy.loginfo('STRMR: cancelling')
            self.vwriter.release()
            #delete video file
            if(os.path.exists(self.filename)):
                os.remove((self.filename))
            self.srec = False
            self.vaction = None


    def run(self, pipe_obj):
        #rospy.init_node('streamr_NODE', anonymous=True)
        rospy.Subscriber(cfgContext['image_topic'], Image, self.captureVideo, queue_size=10)
        rate = rospy.Rate(5)
        while(not rospy.is_shutdown()):
            rospy.loginfo_once('Streamer up and running ...')
            if(pipe_obj.poll()):
                msgObj = pipe_obj.recv()
                #process image capture
                if((msgObj.name == akeys.IMG_STRM) and (msgObj.dataObject)):
                    self.captureImage()
                #process video capture
                if(msgObj.name == akeys.VID_STRM):
                    if((self.srec is False) and (msgObj.dataObject == VIDACTION.START.value)): #if not already recording
                        today = datetime.now()
                        self.filename = self.imgDest + ('videos/%d-%d-%d_%d:%d:%d.avi' %(today.day, today.month, today.year, today.hour, today.minute, today.second))
                        self.vwriter = cv2.VideoWriter(self.filename, self.fourcc, 20.0, (640, 480))
                        self.srec = True
                    self.vaction = msgObj.dataObject
            rate.sleep()
        