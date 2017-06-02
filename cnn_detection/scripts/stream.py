#!/usr/bin/env python

# --------------------------------------------------------
# LSI-Faster R-CNN
# Original work Copyright (c) 2015 Microsoft
# Modified work Copyright 2017 Carlos Guindel
# Licensed under The MIT License [see LICENSE for details]
# Based on a code by Ross Girshick
# Developed at Universidad Carlos III de Madrid
# --------------------------------------------------------

import _init_paths
from fast_rcnn.config import cfg, cfg_from_file
from fast_rcnn.test import im_detect
from utils.timer import Timer
import numpy as np
import caffe, os, sys
import socket, errno, time
import rospy, rospkg
from cnn_detection.msg import Detection
from sensor_msgs.msg import CameraInfo
from rospy.numpy_msg import numpy_msg
from perception_msgs.msg import ObstacleList

info_left = CameraInfo()

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

# == Script ==
PORT = rospy.get_param('port', 3050)

# Connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# rois[0]: stamp
# rois[1]: numpy array (r x 4)
rois = [None, np.empty((0,4), dtype=np.float32)]

def connect_socket():
    global client_socket

    print 'Connecting socket...'
    try:
        client_socket.connect(('localhost', PORT))
    except socket.error:
        return -1
    return 0
    print 'Connected'

def saferecv(sock, n):
    #This is just in case we close the ROS node while receiving
    while True:
        try:
            data = sock.recv(n)
        except socket.error, e:
            if e.errno != errno.EINTR and e.errno != errno.ECONNRESET:
                raise
        else:
            return data

def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = ''
    while len(data) < n and not rospy.is_shutdown():
        packet = saferecv(sock, n - len(data))
        if not packet:
            return None
        data += packet
    return data

def stream(net, im, extra_boxes=np.empty((0,4), dtype=np.float32)):
    """Detect object classes in an image using pre-computed object proposals."""

    # Caffe net must be feeded with extra_boxes
    extra_boxes_param = extra_boxes if extra_boxes.shape[0] > 0 \
                                    else np.zeros((0,4), dtype=np.float32)

    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    if cfg.VIEWPOINTS:
        scores, boxes, orientations = im_detect(net, im, extra_boxes=extra_boxes_param)
    else:
        scores, boxes = im_detect(net, im, extra_boxes=extra_boxes_param)
        #TODO: fix this
        orientations = np.zeros((len(scores), 64), dtype=np.float32)
        orientations[0] = 1
    timer.toc()

    text = '[Classifier] Detection took %.3fs for %d proposals (%d external)' \
            %(timer.total_time, boxes.shape[0], extra_boxes.shape[0])
    rospy.loginfo(text)

    return scores,boxes,orientations

def callback_rois(data):

    print('External ROIs received')

    global rois

    rois[1] = np.empty((0,4), dtype=np.float32)
    for roi in data.obstacles:
        bbox = roi.bbox
        n_bbox = np.array((bbox.x_offset, bbox.y_offset, bbox.x_offset+bbox.width,
                         bbox.y_offset+bbox.height), dtype=np.float32).reshape((1,4))
        rois[1] = np.vstack((rois[1], n_bbox))

    rois[0] = data.header.stamp

def apply_margin(rois, width, height):
    for roi in rois:
        roi[0] = max(0,roi[0]-16)
        roi[1] = max(0,roi[1]-16)
        roi[2] = min(width-1, roi[2]+16)
        roi[3] = min(height-1, roi[3]+16)

def main():

    global client_socket
    global rois
    global chn

    rospy.init_node('stream_classify', anonymous=True)

    # Parameters
    scale = rospy.get_param('~scale', 400)
    margin = rospy.get_param('~margin', 16)

    net_name = rospy.get_param('~net', 'regular')

    #four_channels = rospy.get_param('~four_channels', False)
    use_external_rois = rospy.get_param('~ext_rois', False)
    ex_rois_multiplier = rospy.get_param('~ext_rois_multiplier', 1)
    ex_rois_margin = rospy.get_param('~ext_rois_margin', True)

    # Sanity check
    # if four_channels:
    #   net_name = 'disproj'
    #   assert (use_external_rois == False) # Just because of the prototxt
    # elif use_external_rois:
    #   net_name = 'rois'
    #   assert (four_channels == False) # Just because of the prototxt
    # else:
    #   net_name = 'regular'
    #   assert (use_external_rois == False)
    #   assert (four_channels == False)

    # if four_channels:
    #   chn = 4

    if use_external_rois:
        rois[0] = rospy.get_rostime()
        rois_sub = rospy.Subscriber("/stereo_camera/obstacles",
                                    numpy_msg(ObstacleList),
                                    callback_rois)

    pub = rospy.Publisher('cnn_detection', Detection, queue_size=10)

    text = '[Classifier] Using %s'%(net_name)
    rospy.loginfo(text)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cnn_detection')

    caffemodel = os.path.join(pkg_path,'models',NETS[net_name][1])
    prototxt = os.path.join(pkg_path,'models',NETS[net_name][2])
    configfile = os.path.join(pkg_path,'models',NETS[net_name][3])

    cfg_from_file(configfile)

    CLASSES = cfg.CLASSES
    N_CLASSES = len(CLASSES)

    if cfg.TEST.FOURCHANNELS:
        chn = 4
    else:
        chn = 3

    cfg.TEST.SCALES = [scale]

    if use_external_rois:
        cfg.TEST.EXTERNAL_ROIS = True

    if not os.path.isfile(caffemodel):
        rospy.logerr("%s not found. Please note that you need to download the " \
                    + "LSI trained models separately. Contact the package maintainer(s).", caffemodel)
        return

    caffe.set_mode_gpu()
    caffe.set_device(0)
    cfg.GPU_ID = 0
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    infostr = '\n\nLoaded network {:s}'.format(caffemodel)
    rospy.loginfo(infostr)

    # Warmup on a dummy image
    im = 128 * np.ones((1280, 375, chn), dtype=np.uint8)
    for i in xrange(2):
        _, _, _= stream(net, im)

    con = -1
    while not rospy.is_shutdown() and con<0:
        con=connect_socket()
        time.sleep(1/10.0)

    while not rospy.is_shutdown():
        client_socket.send('r')
        recd = recvall(client_socket, 8)
        if rospy.is_shutdown():
            break
        R_SIZE = int(recd)
        client_socket.send('r')
        data = recvall(client_socket, R_SIZE)

        if data is None:
            continue

        if not rospy.is_shutdown():
            if len(data) == R_SIZE:

                to_send = Detection()

                # Undoing the magic (please check image_sender.py)
                try:
                    to_send.header.stamp.secs = int(data[:10])
                    to_send.header.stamp.nsecs = int(data[10:20])
                    R_WIDTH = int(data[20:24])
                    R_HEIGHT = int(data[24:28])
                except ValueError:
                    rospy.logwarn("[Classifier] No image info embedded in the image")
                    to_send.header.stamp = rospy.Time.now()

                now = rospy.get_rostime()

                try:
                    img = np.fromstring(data, dtype=np.uint8).reshape(R_HEIGHT,R_WIDTH,chn)
                except ValueError:
                    rospy.logerr("Image dimensions do not agree. Expected H: %d, W: %d, chn: %d, data size: %d", R_HEIGHT, R_WIDTH, chn, len(data))
                    break

                if use_external_rois:
                    # Wait until external RoIs are available for the frame
                    while rois[0] < to_send.header.stamp and not rospy.is_shutdown():
                        time.sleep(1/100.0)
                    # Stereo ROIs are tighter than RPN ones
                    if ex_rois_margin:
                        apply_margin(rois[1], R_WIDTH, R_HEIGHT)

                if rois[1].shape[0]<1:
                    scores,boxes,orientations = stream(net, img)
                else:
                    scores,boxes,orientations = stream(net, img, extra_boxes=rois[1])

                rpn_proposals = boxes.shape[0]

                # External ROIs are supposed to be more reliable
                if (use_external_rois):
                    for i in xrange(len(rois[1])):
                        scores[i,:] *= ex_rois_multiplier

                to_send.proposals = rpn_proposals
                to_send.scores = scores.reshape(rpn_proposals*N_CLASSES)
                to_send.boxes = boxes.reshape(rpn_proposals*N_CLASSES*4)
                to_send.sc_orientation = orientations.reshape(rpn_proposals*N_CLASSES*8)
                pub.publish(to_send)

                now = rospy.get_rostime()
                rospy.loginfo("[Classifier] Classified with stamp %.3f at %.3f",
                              to_send.header.stamp.to_sec(), now.to_sec())

    client_socket.send('r')
    client_socket.close()

if __name__ == '__main__':
    main()
