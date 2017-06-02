#!/usr/bin/env python
import _init_paths
import sys
import numpy as np
import time
import math
import rospy
import message_filters
import cv2
from utils.timer import Timer
from cnn_detection.msg import ObstacleList
from cnn_detection.msg import ObjectList
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rospy.numpy_msg import numpy_msg
from perception_msgs.msg import ObstacleList
from perception_msgs.msg import Obstacle
from cv_bridge import CvBridge, CvBridgeError
from fast_rcnn.nms_wrapper import nms

(major, minor, _) = cv2.__version__.split(".")

"""
Please check this variables carefully
"""
CLASSES = ('__background__', # always index 0
         'Car', 'Van', 'Truck', 'Pedestrian',
         'Person sitting', 'Cyclist', 'Tram')

CLASS_COLOR = ((0,0,0),
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

    self.MAP_SIDE = 1000
    self.SCALE = 20

    publish_rois_debug = rospy.get_param('~publish_rois_debug', False)
    self.draw_arrows = rospy.get_param('~draw_arrows', True)
    self.y_offset = rospy.get_param('~y_offset', 0)
    self.roi_height = rospy.get_param('~height', 768)

    self.pub_demo = rospy.Publisher('nice_demo', Image, queue_size=10)
    self.pub_complex_demo = rospy.Publisher('very_nice_demo', Image, queue_size=10)
    self.pub_map = rospy.Publisher('map', Image, queue_size=10)
    if publish_rois_debug:
      self.pub_rois = rospy.Publisher('rois_debug', Image, queue_size=10)
    self.bridge = CvBridge()

    map_sync = message_filters.TimeSynchronizer([
               message_filters.Subscriber("topdown_view", Image, buff_size=self.SIZE*2),
               message_filters.Subscriber("obstacles", numpy_msg(ObstacleList), buff_size=self.SIZE*2)],
               10)

    bboxes_sync = message_filters.TimeSynchronizer([message_filters.Subscriber("/stereo_camera/left/image_rect_color", Image, buff_size=self.SIZE*2),
                  message_filters.Subscriber("filtered_detection", numpy_msg(ObstacleList), buff_size=self.SIZE*2)],
                  10)

    freemap_sync =  message_filters.TimeSynchronizer([message_filters.Subscriber("/stereo_camera/left/image_rect_color", Image, buff_size=self.SIZE*2),
            message_filters.Subscriber("filtered_detection", numpy_msg(ObjectList), buff_size=self.SIZE*2),
            message_filters.Subscriber("free_mask", Image, buff_size=self.SIZE*2)],
            10)

    map_sync.registerCallback(self.map_callback)
    bboxes_sync.registerCallback(self.bboxes_callback)
    freemap_sync.registerCallback(self.freemap_callback)

    if publish_rois_debug:
      externalrois_sync =  message_filters.TimeSynchronizer([message_filters.Subscriber("/stereo_camera/left/image_rect_color", Image, buff_size=self.SIZE*2),
                         message_filters.Subscriber("filtered_detection", numpy_msg(ObjectList), buff_size=self.SIZE*2),
                         message_filters.Subscriber("/stereo_camera/obstacles", numpy_msg(ObstacleList), buff_size=self.SIZE*2),
                         message_filters.Subscriber("/stereo_camera/freeobs_mask", Image, buff_size=self.SIZE*2)],
                         10)
      externalrois_sync.registerCallback(self.externalrois_callback)

  def map_callback(self, data_bv, data_obs):

    texto = "[Painter] Drawing with stamp %.3f at %.3f"% (data_obs.header.stamp.to_sec(),rospy.get_rostime().to_sec())
    rospy.loginfo(texto)

    #Map
    upperview =self.bridge.imgmsg_to_cv2(data_bv, "bgr8");
    things = np.zeros((self.MAP_SIDE,self.MAP_SIDE,3), np.uint8)
    nice_map = np.zeros((self.MAP_SIDE,self.MAP_SIDE,3), np.uint8)

    for obs in data_obs.obstacles:
      if obs.score>self.THRESH[obs.kind]:
        if not math.isnan(obs.location.x):
          cv2.line(things, (int(obs.location.x*self.SCALE+self.MAP_SIDE/2-obs.width/2), int(-obs.location.y*self.SCALE+self.MAP_SIDE)), (int(obs.location.x*self.SCALE+self.MAP_SIDE/2+obs.width/2), int(-obs.location.y*self.SCALE+self.MAP_SIDE)), CLASS_COLOR[obs.kind], 5)

    nice_map = cv2.addWeighted(upperview, 0.5, things, 1, 0)
    self.pub_map.publish(self.bridge.cv2_to_imgmsg(nice_map, "bgr8"))

  def bboxes_callback(self, data_im, data_det):

    #Demo
    image = self.bridge.imgmsg_to_cv2(data_im, "bgr8");
    alpha = np.zeros_like(image)
    nice = np.zeros_like(image)

    cv2.rectangle(alpha,(0, 0),
              (image.shape[1]-1, self.y_offset), (128,128,128), -1)

    cv2.rectangle(alpha,(0, self.y_offset+self.roi_height),
              (image.shape[1]-1, image.shape[0]-1), (128,128,128), -1)

    for obj in data_det.obstacles:
      if obj.score>self.THRESH[obj.kind]:
        bbox = [obj.bbox.x_offset, \
                obj.bbox.y_offset, \
                obj.bbox.x_offset + obj.bbox.width, \
                obj.bbox.y_offset + obj.bbox.height]
        score = obj.score
        angle_bin = int((8 * (obj.alpha/math.pi) -1)/2) # obj.alpha #np.argmax(obj.or_scores)

        cv2.rectangle(image,(int(bbox[0]), int(bbox[1])),
                      (int(bbox[2]), int(bbox[3])),CLASS_COLOR[obj.kind], 2)

        cv2.rectangle(alpha,(int(bbox[0]), int(bbox[1])),
                  (int(bbox[2]), int(bbox[3])),CLASS_COLOR[obj.kind], -1)#2)

        roi_width = int(bbox[2]) - int(bbox[0])
        if(self.draw_arrows):
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
            #cv2.putText(image, '{:s} ({:.0f}%) [{:.1f} {:.1f} {:.1f}]'.format(CLASSES[obs.kind],
            #score*100, obs.location.x, obs.location.y, obs.location.z),
            cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
                (int(bbox[0]), int(bbox[3])+15), cv2.FONT_HERSHEY_DUPLEX,
                0.4, CLASS_COLOR[obj.kind])
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
            #cv2.putText(image, '{:s} ({:.0f}%) [{:.1f} {:.1f} {:.1f}]'.format(CLASSES[obs.kind],
            #score*100, obs.location.x, obs.location.y, obs.location.z),
            cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
                    (int(bbox[0]), int(bbox[1])-10), cv2.FONT_HERSHEY_DUPLEX,
                    0.4, CLASS_COLOR[obj.kind])
          #if int(minor)>8:
          cv2.arrowedLine(image, start_arrow, end_arrow, CLASS_COLOR[obj.kind], 3, cv2.LINE_AA, 0, 0.6)
        #else:
        #  cv2.line(image, start_arrow, end_arrow, CLASS_COLOR[obj.kind], 6)
        else:
          cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
              (int(bbox[0]), int(bbox[3])+15), cv2.FONT_HERSHEY_DUPLEX,
              0.4, CLASS_COLOR[obj.kind])

    #Demo
    nice = cv2.addWeighted(alpha, 0.3, image, 1, 0)
    self.pub_demo.publish(self.bridge.cv2_to_imgmsg(nice, "bgr8"))

  def freemap_callback(self, data_im, data_det, data_mask):

    #Demo
    image = self.bridge.imgmsg_to_cv2(data_im, "bgr8");
    mask = self.bridge.imgmsg_to_cv2(data_mask, "mono8");
    alpha = np.zeros_like(image)
    nice = np.zeros_like(image)
    mask_green = cv2.merge((np.zeros_like(mask),mask,np.zeros_like(mask)))

    for obj in data_det.objects:
      if obj.score>self.THRESH[obj.kind]:
        bbox = obj.bbox
        score = obj.score
        angle_bin = np.argmax(obj.or_scores)

        cv2.rectangle(image,(int(bbox[0]), int(bbox[1])),
                      (int(bbox[2]), int(bbox[3])),CLASS_COLOR[obj.kind], 2)

        cv2.rectangle(alpha,(int(bbox[0]), int(bbox[1])),
                  (int(bbox[2]), int(bbox[3])),CLASS_COLOR[obj.kind], -1)#2)

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
          cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
              (int(bbox[0]), int(bbox[3])+15), cv2.FONT_HERSHEY_DUPLEX,
              0.4, CLASS_COLOR[obj.kind])
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
          cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
                  (int(bbox[0]), int(bbox[1])-10), cv2.FONT_HERSHEY_DUPLEX,
                  0.4, CLASS_COLOR[obj.kind])
        #if int(minor)>8:
        cv2.arrowedLine(image, start_arrow, end_arrow, CLASS_COLOR[obj.kind], 3)
        #else:
        #  cv2.line(image, start_arrow, end_arrow, CLASS_COLOR[obj.kind], 6)

    #Demo
    nice = cv2.addWeighted(alpha, 0.3, image, 1, 0)
    nice = cv2.addWeighted(mask_green, 0.5, nice, 1, 0)
    self.pub_complex_demo.publish(self.bridge.cv2_to_imgmsg(nice, "bgr8"))

  def apply_margin(rois, width, height):
      for roi in rois:
        roi[0] = max(0,roi[0]-16)
        roi[1] = max(0,roi[1]-16)
        roi[2] = min(width-1, roi[2]+16)
        roi[3] = min(height-1, roi[3]+16)

  # External RoIs debug
  def externalrois_callback(self, data_im, data_det, data_jorge, data_freeobsmask):

    image = self.bridge.imgmsg_to_cv2(data_im, "bgr8");
    freeobsmask = self.bridge.imgmsg_to_cv2(data_freeobsmask, "mono8");

    alpha = np.zeros_like(image)
    nice = np.zeros_like(image)

    mask = np.full_like(freeobsmask, 32)

    masked = cv2.bitwise_and(freeobsmask, mask)

    alpha[:,:,0] = masked
    alpha[:,:,2] = masked
    alpha *= 255 / 32

    #jorge_boxes =  np.empty((0,4), dtype=np.float32)
    # cv2.imshow("masked", masked)
    # cv2.waitKey(4)

    n_external = len(data_jorge.obstacles)

    for obj in data_jorge.obstacles:
      bbox = obj.bbox
      # cv2.rectangle(image,(int(bbox.x_offset), int(bbox.y_offset)),
      #                 (int(bbox.x_offset+bbox.width), int(bbox.y_offset+bbox.height)),(255,255,255), 5)
      cv2.rectangle(alpha,(int(bbox.x_offset-16), int(bbox.y_offset-16)),
                      (int(bbox.x_offset+bbox.width+16), int(bbox.y_offset+bbox.height+16)),(0,0,255), -1)

    n_rpn_rois = 0
    for obj in data_det.objects:
      n_rpn_rois += 1

      # TODO: Ablation experiments
      # if n_rpn_rois > n_external:
      #   break

      bbox = obj.bbox
      #match=np.where(4== (0== (jorge_boxes-bbox)).sum(1))
      #if len(match[0]>0):
      #  width = 8
      #else:
      #  width=1
      width=1

      # print obj.bbox[0], obj.bbox[1], obj.bbox[2], obj.bbox[3]
      # print obj.kind

      if obj.score>self.THRESH[obj.kind]:

        score = obj.score
        angle_bin = np.argmax(obj.or_scores)

        cv2.rectangle(image,(int(bbox[0]), int(bbox[1])),
                      (int(bbox[2]), int(bbox[3])),CLASS_COLOR[obj.kind], width)

        cv2.rectangle(alpha,(int(bbox[0]), int(bbox[1])),
                  (int(bbox[2]), int(bbox[3])),CLASS_COLOR[obj.kind], -1)#2)

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
          cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
              (int(bbox[0]), int(bbox[3])+15), cv2.FONT_HERSHEY_DUPLEX,
              0.4, CLASS_COLOR[obj.kind])
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
          cv2.putText(image, '{:s} ({:.0f}%)'.format(CLASSES[obj.kind], score*100),
                  (int(bbox[0]), int(bbox[1])-10), cv2.FONT_HERSHEY_DUPLEX,
                  0.4, CLASS_COLOR[obj.kind])
        if int(minor)>8:
          cv2.arrowedLine(image, start_arrow, end_arrow, CLASS_COLOR[obj.kind], 3)
        else:
          cv2.line(image, start_arrow, end_arrow, CLASS_COLOR[obj.kind], 6)

      else:
        bbox = obj.bbox
        # cv2.rectangle(image,(int(bbox[0]), int(bbox[1])),
        #               (int(bbox[2]), int(bbox[3])),(0,0,0), width)

    #Demo
    nice = cv2.addWeighted(alpha, 0.3, image, 1, 0)
    self.pub_rois.publish(self.bridge.cv2_to_imgmsg(nice, "bgr8"))

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
