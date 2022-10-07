#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class viewer:

  def __init__(self):
    self.bridge = CvBridge()
    self.camera_info_sub = message_filters.Subscriber('/stereo_inertial_publisher/stereo/camera_info', CameraInfo)
    self.image_sub = rospy.Subscriber("/stereo_inertial_publisher/stereo/depth", Image, self.callback)
    self.rgb_sub = rospy.Subscriber("/stereo_inertial_publisher/color/image", Image, self.callback)
    self.text_sub=rospy.Subscriber("/pallet_detector/pallet",String,self.callback)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub,self.rgb_sub, self.text_sub,self.camera_info_sub], queue_size=10, slop=0.5)
    self.ts.registerCallback(self.callback)

  def callback(self,data,rgb,box,camera_info):
    try:
      depth = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image=rgb
      box=box
      xmin,xmax,ymin,ymax=box.split(' ')
      xmin=float(xmin)
      xmax=float(xmax)
      ymin=float(ymin)
      ymax=float(ymax)
      	
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      gray=cv2.medianBlur(gray,5) #median blur to remove imperfections
      edges = cv2.Canny(gray,50,150,apertureSize=3)#edge detection to get best corners
      gray = np.float32(edges)
      dst = cv.cornerHarris(gray,2,9,0.04)
#result is dilated for marking the corners, not important
      dst = cv.dilate(dst,None)
      corners=[]
      i_val=[]
      j_val=[]
      thresh = 0.1*dst.max()
      for j in range(0, dst.shape[0]):
        for i in range(0, dst.shape[1]):
        if(dst[j,i] > thresh):
            # image, center pt, radius, color, thickness
            i_val.append(i)
            j_val.append(j)
            
      x=[]
      y=[]
      for j in i_val:
          if j in range(xmin,xmax):
              x.append(j)
              
      for j in j_val:
          if j in range(ymin,ymax):
              y.append(j)
              
      corners=[]
      corners.append((i_val[np.argmax(i_val)],j_val[np.argmax(i_val)]))
      corners.append((i_val[np.argmin(i_val)],j_val[np.argmax(i_val)]))
      corners.append((i_val[np.argmax(i_val)],j_val[np.argmin(i_val)]))
      corners.append((i_val[np.argmin(i_val)],j_val[np.argmin(i_val)]))
      
          
      camera_info_K = np.array(camera_info.K)
      
      # Intrinsic camera matrix for the raw (distorted) images.
      #     [fx  0 cx]
      # K = [ 0 fy cy]
      #     [ 0  0  1]
    
      m_fx = camera_info.K[0];
      m_fy = camera_info.K[4];
      m_cx = camera_info.K[2];
      m_cy = camera_info.K[5];
      inv_fx = 1. / m_fx;
      inv_fy = 1. / m_fy;
      depths=[]
      for i in range(4):
        roi_depth = cv_image[corners[i][1]-5:corners[i][1]+5, corners[i][0]-5:corners[i][0]+5]
        n = 0
        sum = 0
        for i in range(0,roi_depth.shape[0]):
            for j in range(0,roi_depth.shape[1]):
                value = roi_depth.item(i, j)
                if value > 0.:
                    n = n + 1
                    sum = sum + value
        depths.append(sum/n)
        
      cordinates=[]
      for i in range(4):
        
            point_z = depths[i] * 0.001; # distance in meters
            point_x = ((corners[i][0] + w/2) - m_cx) * point_z * inv_fx;
            point_y = ((corners[i][1] + h/2) - m_cy) * point_z * inv_fy;
            cordinates.append((point_x,point_y,point_z))
                
    
      
    except CvBridgeError as e:
      print(e)    

    
   

def main(args):
  v = viewer()
  rospy.init_node('viewer_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)