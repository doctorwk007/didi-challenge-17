import os
import sys
import cv2
import rosbag
import math
import argparse
from parse_tracklet import *

cam_frames = []
velo_frames = []

def parse_camera_tracklets(tracklet_list):
    global cam_gt
    for tracklet in tracklet_list:
        index = 0
        for pose in tracklet:
            # img_number = tracklet.first_frame + index
            # centroid = [pose[0][0], pose[0][1]]
            length = tracklet.size[2]
            width = tracklet.size[1]
            # yaw = pose[1][2]

            cam_gt.append([pose[0][0], pose[0][1], pose[1][2], length, width, tracklet.object_type])
            index += 1


def interpolate_velo_tracklets(images_path):
    camera_array = np.array(cam_frames)
    indices = np.searchsorted(camera_array, velo_frames)

    for i in xrange(0, indices.size):
        if indices[i]-1 > -1 and indices[i] < len(cam_frames):
            prev_time = cam_frames[indices[i]-1]
            velo_time = velo_frames[i]
            next_time = cam_frames[indices[i]]

            prev_gt = cam_gt[indices[i]-1]
            next_gt = cam_gt[indices[i]]

            # Interpolated values
            coefficient = float((velo_time-prev_time))/(next_time-prev_time)
            interpolated_x = prev_gt[0] * (1-coefficient) + next_gt[0] * coefficient
            interpolated_y = prev_gt[1] * (1-coefficient) + next_gt[1] * coefficient
            centroid = [interpolated_x, interpolated_y]
            yaw = prev_gt[2] * (1-coefficient) + next_gt[2] * coefficient
            length = prev_gt[3]
            width = prev_gt[4]
            category = prev_gt[5]

            # Compute the four vertexes coordinates
            corners = np.array([[centroid[0]-length/2., centroid[1]+width/2.],
                               [centroid[0]+length/2., centroid[1]+width/2.],
                               [centroid[0]+length/2., centroid[1]-width/2.],
                               [centroid[0]-length/2., centroid[1]-width/2.]])

            # Compute rotation matrix
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([[c, -s], [s, c]])

            # Rotate all corners at once by yaw
            rotated_corners = np.dot(corners-centroid, R) + centroid

            for j in xrange(1, nrotation+1):
                # Compute rotation matrix
                rotation_angle = (j-1) * math.pi/2
                c, s = np.cos(rotation_angle), np.sin(rotation_angle)
                R = np.array([[c, -s], [s, c]])
                final_corners = np.dot(rotated_corners, R)
                leftpx = bvsize / 2 + min(-final_corners[:, 1]) / bvres
                rightpx = bvsize / 2 + max(-final_corners[:, 1]) / bvres
                bottompx = bvsize / 2 - min(final_corners[:, 0]) / bvres
                toppx = bvsize / 2 - max(final_corners[:, 0]) / bvres

                current_yaw = yaw-rotation_angle
                # Angle should be in -pi < object_orientation < pi
                if current_yaw > math.pi:
                    current_yaw = current_yaw - 2 * math.pi
                elif current_yaw < -math.pi:
                    current_yaw = current_yaw + 2 * math.pi

                prefix = j
                if nrotation == 1:
                    prefix = 0

                if visualization:
                    # Show result
                    img_name = '%s%d%05d.png' % (images_path, prefix, i)
                    print img_name
                    imagen = cv2.imread(img_name)
                    cv2.rectangle(imagen, (int(leftpx), int(toppx)), (int(rightpx), int(bottompx)), (0, 255, 0), -1)
                    cv2.imshow("Labels", imagen)
                    cv2.waitKey(0)
                else:
                    filename = '%d%05d.txt' % (prefix, i)
                    print 'Created %s' % filename
                    with open(os.path.join(images_path, filename), 'a') as f:
                        line = '{:s} -1 -1 {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} -1 -1 -1 -1000 -1000 -1000 -10\n'.format(
                            category, current_yaw, leftpx, toppx, rightpx, bottompx)
                        f.write(line)
                        f.close()


def main():
    parser = argparse.ArgumentParser(
        description='Generate KITTI ground truth from tracklet_labels.xml file.')
    parser.add_argument('-t', '--tracklets', type=str, nargs='?', default='tracklet_labels.xml',
                        help='Tracklet label path')
    parser.add_argument('-i', '--images_path', type=str, nargs='?', help='Path to images (viz only)')
    parser.add_argument('-b', '--bag', type=str, nargs='?', help='Sequence .bag filename')
    parser.add_argument('-s', '--birdview_size', type=int, default=700, help='Size of the image in pixels')
    parser.add_argument('-r', '--birdview_resolution', type=float, default=0.05, help='Pixel resolution in meters')
    parser.add_argument('-nr', '--number_rotation', type=int, default=1,
                        help='Number (N) of rotations for data augmentation. \
                        Each frame and its labels will be rotated by 2*pi/N')
    parser.add_argument('--viz', dest='visualization', action='store_true',
                        help='Use visualization of the labels. No .txt files are created ')

    args = parser.parse_args()
    tracklets_file = args.tracklets
    if not os.path.exists(tracklets_file):
        sys.stderr.write('Error: Tracklets file %s not found.\n' % tracklets_file)
        exit(-1)

    tracklets_list = parse_xml(tracklets_file)
    if not tracklets_list:
        sys.stderr.write('Error: No Tracklets parsed for predictions.\n')
        exit(-1)

    images_path = args.images_path
    if not os.path.exists(images_path):
        sys.stderr.write('Invalid images path.\n')
        exit(-1)

    global bvsize, bvres, visualization, nrotation
    bvsize = args.birdview_size
    bvres = args.birdview_resolution
    visualization = args.visualization
    nrotation = args.number_rotation

    bag_file = args.bag
    bag = rosbag.Bag(bag_file, 'r')

    # Store camera frame_idx-to-stamp correspondence
    frame_number = 0
    global cam_frames
    for topic, msg, t in bag.read_messages(topics=["/image_raw"]):
        cam_frames.append(msg.header.stamp.to_nsec())
        frame_number += 1

    global cam_gt
    cam_gt = []

    velo_number = 0
    global velo_frames
    for topic, msg, t in bag.read_messages(topics=["/velodyne_packets"]):
        velo_frames.append(msg.header.stamp.to_nsec())
        velo_number += 1

    bag.close()

    parse_camera_tracklets(tracklets_list)
    interpolate_velo_tracklets(images_path)

if __name__ == '__main__':
    main()
