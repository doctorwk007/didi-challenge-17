import os
import sys
import rospy
import tf
import rosbag
import argparse
from parse_tracklet import *
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from shutil import copyfile

frames = []


def write_tracklets(tracklet_list, topic, bag_file, color, as_sphere):
    for tracklet in tracklet_list:
        index = 0
        if as_sphere:
            sphere_diameter = max(tracklet.size)
        for pose in tracklet:
            mkr = Marker()
            mkr.header.frame_id = "velodyne"
            mkr.header.stamp = frames[tracklet.first_frame+index]
            mkr.action = 0
            mkr.pose.position.x = pose[0][0]
            mkr.pose.position.y = pose[0][1]
            mkr.pose.position.z = pose[0][2]
            if not as_sphere:  #
                mkr.type = 1  # sphere
                quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[1][2])
                mkr.pose.orientation.x = quaternion[0]
                mkr.pose.orientation.y = quaternion[1]
                mkr.pose.orientation.z = quaternion[2]
                mkr.pose.orientation.w = quaternion[3]
                mkr.scale.x = tracklet.size[2]
                mkr.scale.y = tracklet.size[1]
                mkr.scale.z = tracklet.size[0]
            else:
                mkr.type = 2  # sphere
                mkr.pose.orientation.x = 0
                mkr.pose.orientation.y = 0
                mkr.pose.orientation.z = 0
                mkr.pose.orientation.w = 1
                mkr.scale.x = sphere_diameter
                mkr.scale.y = sphere_diameter
                mkr.scale.z = sphere_diameter
            mkr.color = color
            mkr.lifetime = rospy.Duration(0.04)
            bag_file.write(topic, mkr, mkr.header.stamp)
            index += 1


def main():
    # rospy.init_node('tracklets_to_marker', anonymous=True)

    parser = argparse.ArgumentParser(
        description='Generate markers for two tracklet files and add them to sequence .bag file.')
    parser.add_argument('-p', '--prediction', type=str, nargs='?', default='tracklet_labels.xml',
                        help='Predicted tracklet label filename')
    parser.add_argument('-gt', '--groundtruth', type=str, nargs='?', default='tracklet_labels_gt.xml',
                        help='Groundtruth tracklet label filename')
    parser.add_argument('-b', '--bag', type=str, nargs='?', help='Sequence .bag filename')
    parser.add_argument('--sphere', dest='as_sphere', action='store_true', help='Markers as spheres')

    args = parser.parse_args()
    as_sphere = args.as_sphere
    pred_file = args.prediction
    if not os.path.exists(pred_file):
        sys.stderr.write('Error: Prediction file %s not found.\n' % pred_file)
        exit(-1)

    gt_file = args.groundtruth
    if not os.path.exists(gt_file):
        sys.stderr.write('Error: Ground-truth file %s not found.\n' % gt_file)
        exit(-1)

    orig_bag_file = args.bag
    if not os.path.exists(orig_bag_file):
        sys.stderr.write('Error: Ground-truth file %s not found.\n' % orig_bag_file)
        exit(-1)

    pred_tracklets = parse_xml(pred_file)
    if not pred_tracklets:
        sys.stderr.write('Error: No Tracklets parsed for predictions.\n')
        exit(-1)

    gt_tracklets = parse_xml(gt_file)
    if not gt_tracklets:
        sys.stderr.write('Error: No Tracklets parsed for ground truth.\n')
        exit(-1)

    # Make copy and open bag file to write
    bag_file = os.path.dirname(orig_bag_file) + "/marker_" + os.path.basename(orig_bag_file)

    print('Copying original bag file in %s to add prediction and ground-truth markers' % bag_file)
    copyfile(orig_bag_file, bag_file)
    bag = rosbag.Bag(bag_file, 'a')

    # Store camera frame_idx-to-stamp correspondence
    frame_number = 0
    global frames
    for topic, msg, t in bag.read_messages(topics=["/image_raw"]):
        frames.append(msg.header.stamp)
        frame_number += 1

    # Write prediction marker msgs synchronized with camera frames
    color = ColorRGBA()
    color.r, color.g, color.b, color.a = 1.0, 0.0, 0.0, 0.5
    write_tracklets(pred_tracklets, "prediction_marker", bag, color, as_sphere)

    # Write ground truth marker msgs synchronized with camera frames
    color.r, color.g, color.b, color.a = 0.0, 1.0, 0.0, 0.5
    write_tracklets(gt_tracklets, "gt_marker", bag, color, as_sphere)

    bag.close()


if __name__ == '__main__':
    main()
