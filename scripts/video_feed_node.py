#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
import numpy
import time
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

last_time = time.time()

def ShowImage(image):
  global last_time
  # image is compressed, decompress and then show
  np_arr = numpy.frombuffer(image.data, numpy.uint8)
  image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  rgbImage = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
  now = time.time()
  print(image_np.shape)
  cv2.imshow("feed_window", rgbImage)
  cv2.setWindowTitle("feed_window", 'Feed ({:.2f} fps)'.format(1 / (now - last_time)))  
  cv2.waitKey(100)
  last_time = now

  
def main():
 
  sub = rospy.Subscriber('/video_frames',CompressedImage, ShowImage, queue_size=1)
  rospy.init_node('video_feed_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # While ROS is still running.
  while not rospy.is_shutdown():
      # Sleep just enough to maintain the desired rate
      rate.sleep()

  cv2.destroyAllWindows()
         
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    cv2.destroyAllWindows()