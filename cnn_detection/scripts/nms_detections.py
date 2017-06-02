#!/usr/bin/env python
import _init_paths
from fast_rcnn.nms_wrapper import nms
from fast_rcnn.config import cfg, cfg_from_file
from utils.timer import Timer
import numpy as np
import time, math, os
import cv2
import rospy, rospkg
import message_filters
from rospy.numpy_msg import numpy_msg
from cnn_detection.msg import Detection
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
from perception_msgs.msg import Obstacle
from perception_msgs.msg import ObstacleList
from cv_bridge import CvBridge, CvBridgeError

# KITTI classes (for reference)
# 0:Background,     1:Car,      2:Van,  3:Truck,    4:Pedestrian
# 5:Person_sitting  6:Cyclist   7:Tram  8:Misc

# == Common nets definition ==
NETS = {'regular': ('kitti/VGG16',
            'vgg16.caffemodel',
            'regular.prototxt', 'regular.yml'),
        'rois': ('kitti/VGG16',
            'vgg16.caffemodel',
            'rois.prototxt', 'rois.yml'),
        'disproj': ('kitti/VGG16',
            'fourchannels.caffemodel',
            'fourchannels.prototxt', 'fourchannels.yml'),
        'didi': ('kitti/VGG16',
            'didi.caffemodel',
            'didi.prototxt', 'didi.yml')
        }

# == Node ==
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('cnn_detection')

net_name = rospy.get_param('~net', 'regular')

CLASSES = []
N_CLASSES = 0
N_BINS = 0

CONF_THRESH = 0.6
NMS_THRESH = 0.3

pub_filter = rospy.Publisher('filtered_detection', ObstacleList, queue_size=10)
bridge = CvBridge()

y_offset = 0
y_offset_list = []

def callback(data):

    timer = Timer()
    timer.tic()

    now = rospy.get_rostime()

    rpn_proposals = data.proposals

    scores = data.scores.reshape(rpn_proposals,N_CLASSES)
    boxes = data.boxes.reshape(rpn_proposals,N_CLASSES*4)
    sc_orientation = data.sc_orientation.reshape(rpn_proposals,N_CLASSES*N_BINS)

    toSend = ObstacleList()
    toSend.header = data.header

    object_list = []

    for cls_ind in xrange(N_CLASSES-1):

        # if cls_ind == 2:
        #     continue

        cls_ind += 1 # skip background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        cls_kind = np.empty(scores.shape[0])
        cls_kind.fill(cls_ind)
        cls_sc_orient = sc_orientation[:, N_BINS*cls_ind:N_BINS*cls_ind+N_BINS]

        #TODO: Didi
        #   if cls_ind == 1:
        #     # Vans and Cars are supressed together
        #     # TODO: This should be generic
        #     cls_ind += 1
        #     cls_boxes = np.vstack((cls_boxes, boxes[:, 4*cls_ind:4*(cls_ind + 1)]))
        #     cls_scores = np.hstack((cls_scores, scores[:, cls_ind]))
        #     cls_sc_orient = np.vstack((cls_sc_orient, sc_orientation))
        #     cls_kind_2 = np.empty(scores.shape[0])
        #     cls_kind_2.fill(cls_ind)
        #     cls_kind = np.hstack((cls_kind, cls_kind_2))

        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis],
                          cls_kind[:, np.newaxis])).astype(np.float32)

        keep = nms(dets[:,:-1], NMS_THRESH)

        dets = dets[keep, :]
        orients = cls_sc_orient[keep, :]

        for ind, det in enumerate(dets):
            obj = Obstacle()

            temp_bbox = det[0:4].tolist()
            bbox = [coord+y_offset_list[ix] for ix, coord in enumerate(temp_bbox)]

            obj.kind_name = CLASSES[cls_ind]
            obj.kind = cls_ind
            obj.bbox.x_offset = bbox[0]
            obj.bbox.y_offset = bbox[1]
            obj.bbox.height = bbox[3]-bbox[1]
            obj.bbox.width = bbox[2]-bbox[0]
            obj.score = det[4]
            obj.alpha = math.pi * (2* np.argmax(orients[ind,:]) + 1)/N_BINS
            #np.argmax(orients[ind,:])
            #obj.yaw = math.pi * (2* np.argmax(orients[ind,:]) + 1)/N_BINS

            object_list.append(obj)

    toSend.obstacles = object_list

    pub_filter.publish(toSend)

    timer.toc()
    text = "[NMS] Filtered with stamp %.3f at %.3f: took %.3fs" \
        % (data.header.stamp.to_sec(),now.to_sec(),timer.total_time)
    rospy.loginfo(text)

def listener():

    global y_offset
    global y_offset_list
    global net_name
    global CLASSES
    global N_CLASSES
    global N_BINS

    rospy.init_node('nms_detections', anonymous=True)

    net_name = rospy.get_param('~net', 'regular')

    text = '[NMS] Using %s'%(net_name)
    rospy.loginfo(text)

    y_offset = rospy.get_param('~y_offset', 0)
    y_offset_list = [0, y_offset, 0, y_offset]

    det_sub = rospy.Subscriber("cnn_detection", numpy_msg(Detection), callback)

    configfile = os.path.join(pkg_path,'models',NETS[net_name][3])

    cfg_from_file(configfile)

    CLASSES = cfg.CLASSES

    N_CLASSES = len(CLASSES)
    N_BINS = cfg.VIEWP_BINS

    rospy.spin()

if __name__ == '__main__':
    listener()
