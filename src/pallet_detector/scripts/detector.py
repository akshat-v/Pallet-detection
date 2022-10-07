#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import torch 
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class pallet_detector:

  def __init__(self):
    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber("/stereo_inertial_publisher/color/image", Image, self.callback)
    
    self.pub = rospy.Publisher('/pallet_detector/pallet', String, queue_size=25)	
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='pallet.pt') 
    

  def callback(self, rgb_data):
    
    try:
      img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
      #face_cascade = cv2.CascadeClassifier('/home/bloisi/catkin_ws/src/unibas_face_detector/haarcascade/haarcascade_frontalface_default.xml')
      #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      #faces = face_cascade.detectMultiScale(gray, 1.3, 5)
      result=model(img)
      result=result.pandas().xyxy[0]
      ymax=result['ymax'][0]
      ymin=result['ymin'][0]
      xmax=result['xmax'][0]
      xmin=result['xmin'][0]
      box=str(xmin)+' '+str(xmax)+' '+str(ymin)+' '+str(ymax)
      
      
            
    except CvBridgeError as e:
      print(e)

    
    try:
      
      self.pub.publish(box)
    except CvBridgeError as e:
      print(e)
    

def main(args):
  fd = pallet_detector()
  rospy.init_node('pallet_detector_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)