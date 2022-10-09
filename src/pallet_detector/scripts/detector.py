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
    
    self.image_sub = rospy.Subscriber("/stereo_inertial_publisher/color/image", Image, self.callback) #topic to get the rgb image
    
    self.pub = rospy.Publisher('/pallet_detector/pallet', String, queue_size=25)	#publisher to publish the cordinates of the detected pallet
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='pallet.pt')   #loading the custom model
    

  def callback(self, rgb_data):
    
    try:
      img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
      
      result=model(img)
      result=result.pandas().xyxy[0] #converting the result to a dataframe
      ymax=result['ymax'][0]  # getting the corner points of the bounding box of the first pallet
      ymin=result['ymin'][0]
      xmax=result['xmax'][0]
      xmin=result['xmin'][0]
      box=str(xmin)+' '+str(xmax)+' '+str(ymin)+' '+str(ymax) #string to publish the cordinates 
      
      
            
    except CvBridgeError as e:
      print(e)

    
    try:
      
      self.pub.publish(box) #publishing the values
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
