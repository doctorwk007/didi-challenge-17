#!/usr/bin/env python
import sys
import numpy as np
import time
import math
import rospy
import message_filters
import cv2
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from perception_msgs.msg import ObstacleList
from perception_msgs.msg import Obstacle
from cv_bridge import CvBridge

"""
Please check this variables carefully
"""
CLASSES = ('__background__', # always index 0
         'Car', 'Van', 'Truck', 'Pedestrian',
         'Person sitting', 'Cyclist', 'Tram')

CLASS_COLOR = ((255,255,255),
        (0,0,255), (0,128,255),(0,192,255),(255,255,0),
        (128,128,0),(0,255,255), (255,255,255), (0,0,0))

CLASS_THRESHOLDS = (0,
        0.5,0.5,0.5,0.5,
        0.5,0.5,0.5)

N_CLASSES = len(CLASSES)

class obstacle_painter:
  def __init__(self):
    self.SIZE = 1280*960*3 # Just to set the buffer_size

    self.THRESH = CLASS_THRESHOLDS

    self.draw_arrows = rospy.get_param('~draw_arrows', True)

    self.pub_demo = rospy.Publisher('bboxes', Image, queue_size=10)

    self.bridge = CvBridge()

    bboxes_sync = message_filters.TimeSynchronizer([message_filters.Subscriber("image", Image, buff_size=self.SIZE*2),
                  message_filters.Subscriber("detection", numpy_msg(ObstacleList), buff_size=self.SIZE*2)],
                  10)

    bboxes_sync.registerCallback(self.bboxes_callback)

  def compute_arrows(self, bbox):
    roi_width = int(bbox[2]) - int(bbox[0])
    if angle_bin > 3:
      start_arrow = (int(bbox[0])+roi_width/2, int(bbox[1])-5)
      if angle_bin == 4:
          end_arrow = (int(bbox[0])+roi_width/2-48, int(bbox[1])-25)
      elif angle_bin == 5:
          end_arrow = (int(bbox[0])+roi_width/2-8, int(bbox[1])-25)
      elif angle_bin == 6:
          end_arrow = (int(bbox[0])+roi_width/2+8, int(bbox[1])-25)
      elif angle_bin == 7:
          end_arrow = (int(bbox[0])+roi_width/2+48, int(bbox[1])-25)
      pos = 0
    else:
      start_arrow = (int(bbox[0])+roi_width/2, int(bbox[3])+5)
      if angle_bin == 0:
          end_arrow = (int(bbox[0])+roi_width/2+48, int(bbox[3])+25)
      elif angle_bin == 1:
          end_arrow = (int(bbox[0])+roi_width/2+8, int(bbox[3])+25)
      elif angle_bin == 2:
          end_arrow = (int(bbox[0])+roi_width/2-8, int(bbox[3])+25)
      elif angle_bin == 3:
          end_arrow = (int(bbox[0])+roi_width/2-48, int(bbox[3])+25)
      pos = 1

    return start_arrow, end_arrow, pos

  def bboxes_callback(self, data_im, data_det):

    #Demo
    image = self.bridge.imgmsg_to_cv2(data_im, "bgr8");
    alpha = np.zeros_like(image)
    nice = np.zeros_like(image)

    for obj in data_det.obstacles:
      if obj.score<1 or obj.score>self.THRESH[obj.kind]:
        bbox = [obj.bbox.x_offset, \
                obj.bbox.y_offset, \
                obj.bbox.x_offset + obj.bbox.width, \
                obj.bbox.y_offset + obj.bbox.height]

        kind = obj.kind
        score = obj.score

        if obj.alpha>0:
          start_arrow, end_arrow, pos = compute_arrows(bbox)
          cv2.arrowedLine(image, start_arrow, end_arrow, CLASS_COLOR[kind], 3, cv2.LINE_AA, 0, 0.6)
        else:
          pos = 0

        # bounding box
        cv2.rectangle(image,(int(bbox[0]), int(bbox[1])),
                      (int(bbox[2]), int(bbox[3])),CLASS_COLOR[kind], 2)

        if score>0:
          # translucid filling
          cv2.rectangle(alpha,(int(bbox[0]), int(bbox[1])),
                    (int(bbox[2]), int(bbox[3])),CLASS_COLOR[kind], -1)#2)

          cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[kind], score*100),
                      (int(bbox[0]), int(bbox[3])+15-25*pos), cv2.FONT_HERSHEY_DUPLEX,
                      0.4, CLASS_COLOR[kind])

    #Demo
    nice = cv2.addWeighted(alpha, 0.3, image, 1, 0)
    self.pub_demo.publish(self.bridge.cv2_to_imgmsg(nice, "bgr8"))

def main(args):
  #Initializes and cleanup ros node
  rospy.init_node('obstacle_painter', anonymous=True)
  op = obstacle_painter()

  try:
    rospy.loginfo("[Painter] Ready")
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down py Obstacle Painter")

if __name__ == '__main__':
    main(sys.argv)
