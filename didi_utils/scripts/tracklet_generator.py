#!/usr/bin/env python
import os
import rospy
from tracklet_lib import *
from perception_msgs.msg import ObstacleList
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from collections import defaultdict
import pandas as pd
import PyKDL as kd
import functools
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

current_frame = -1
frames = {}
tracklets_queue = []
collection = TrackletCollection()
last_frame = -1
aux_poses = []
aux_frames = []
id_remap = []

camera_cols = ["timestamp"]
camera_dict = defaultdict(list)
rtk_cols = ["timestamp", "tx", "ty", "tz", "rx", "ry", "rz"]
cap_rear_rtk_dict = defaultdict(list)
cap_front_rtk_dict = defaultdict(list)
obs_dict = {}
rtk_cap_obs = {}


def image_callback(image):
    global current_frame
    global frames
    current_frame+=1
    # frames[image.header.stamp.to_nsec()] = current_frame
    frames[str(image.header.stamp.to_nsec())[:-6]] = current_frame
    timestamp = image.header.stamp.to_nsec()
    camera_dict["timestamp"].append(timestamp)
    # print str(image.header.stamp.to_nsec())[:-6], " ", frames[str(image.header.stamp.to_nsec())[:-6]]


def rear_rtk_callback(rtk):
    global cap_rear_rtk_dict
    rtk2dict(rtk, cap_rear_rtk_dict)


def front_rtk_callback(rtk):
    global cap_front_rtk_dict
    rtk2dict(rtk, cap_front_rtk_dict)


def rtk2dict(msg, rtk_dict):
    rtk_dict["timestamp"].append(msg.header.stamp.to_nsec())
    rtk_dict["tx"].append(msg.pose.pose.position.x)
    rtk_dict["ty"].append(msg.pose.pose.position.y)
    rtk_dict["tz"].append(msg.pose.pose.position.z)
    rotq = kd.Rotation.Quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    rot_xyz = rotq.GetRPY()
    rtk_dict["rx"].append(0.0) #rot_xyz[0]
    rtk_dict["ry"].append(0.0) #rot_xyz[1]
    rtk_dict["rz"].append(rot_xyz[2])


def dict_to_vect(di):
    return kd.Vector(di['tx'], di['ty'], di['tz'])


def get_yaw(p1, p2):
    return math.atan2(p1[1] - p2[1], p1[0] - p2[0])


def list_to_vect(li):
    return kd.Vector(li[0], li[1], li[2])


def frame_to_dict(frame):
    r, p, y = frame.M.GetRPY()
    return dict(tx=frame.p[0], ty=frame.p[1], tz=frame.p[2], rx=r, ry=p, rz=y)


def get_velo_pos(front, rear):
    front_v = dict_to_vect(front)
    rear_v = dict_to_vect(rear)

    # Gps-Velodyne tf
    lrg_to_gps = [0.4572, 0.0, 1.2192]
    lrg_to_velodyne = [1.5494, 0.0, 1.27]
    gps_to_velodyne = np.subtract(lrg_to_velodyne, lrg_to_gps)

    yaw = get_yaw(front_v, rear_v)
    rot_z = kd.Rotation.RotZ(yaw)

    # Get capture vehicle velodyne global position
    alt_pos = front_v + rot_z * list_to_vect(gps_to_velodyne)
    return alt_pos, yaw


def get_global_obstacle_pos(
        front,
        rear,
        relative_obs):
    # Get capture vehicle velodyne global position
    capture_pos, yaw = get_velo_pos(front, rear)
    rot_z = kd.Rotation.RotZ(yaw)

    # Compute obstacle global coords
    rtk_obs = capture_pos + rot_z * dict_to_vect(relative_obs)
    return frame_to_dict(kd.Frame(kd.Rotation(), rtk_obs))


def get_local_obstacle_pos(
        front,
        rear,
        obstacle_pos):
    obs_v = dict_to_vect(obstacle_pos)
    capture_v, yaw = get_velo_pos(front, rear)
    rot_z = kd.Rotation.RotZ(-yaw)

    diff = obs_v - capture_v
    res = rot_z * diff
    return frame_to_dict(kd.Frame(kd.Rotation(), res))


def interpolate_to_camera(camera_df, other_dfs, filter_cols=[]):
    if not isinstance(other_dfs, list):
        other_dfs = [other_dfs]
    if not isinstance(camera_df.index, pd.DatetimeIndex):
        print('Error: Camera dataframe needs to be indexed by timestamp for interpolation')
        return pd.DataFrame()

    for o in other_dfs:
        o['timestamp'] = pd.to_datetime(o['timestamp'])
        o.set_index(['timestamp'], inplace=True)
        o.index.rename('index', inplace=True)

    merged = functools.reduce(lambda left, right: pd.merge(
        left, right, how='outer', left_index=True, right_index=True), [camera_df] + other_dfs)
    merged.interpolate(method='time', inplace=True, limit=100, limit_direction='both')

    filtered = merged.loc[camera_df.index]  # back to only camera rows
    filtered.fillna(0.0, inplace=True)
    filtered['timestamp'] = filtered.index.astype('int')  # add back original timestamp integer col
    if filter_cols:
        if not 'timestamp' in filter_cols:
            filter_cols += ['timestamp']
        filtered = filtered[filter_cols]

    return filtered


def obstacle_callback(obstacle_list):
    # Check if obstacles in msg and exit otherwise
    if len(obstacle_list.obstacles) < 1:
        return

    global frames
    global collection
    global last_frame
    camera_frame = frames[str(obstacle_list.header.stamp.to_nsec())[:-6]]
    hole = False
    if (camera_frame-last_frame) != 1:
        hole = True
        # print ('\nHole of %d detected\n' % (camera_frame-last_frame))
    # print ('Detection at frame %d' % current_frame)
    # print ('received at %d' % camera_frame)
    for obstacle in obstacle_list.obstacles:
        pose = {}
        pose['tx'] = obstacle.location.x
        pose['ty'] = obstacle.location.y
        pose['tz'] = obstacle.location.z
        pose['rx'] = 0
        pose['ry'] = 0
        pose['rz'] = obstacle.alpha  # TODO CHANGE to obstacle.yaw

        while len(id_remap) <= obstacle.id:  # Loop to add aux poses for every agent
            id_remap.append([])

        if type(id_remap[obstacle.id]) is list:  # First time
            id_remap[obstacle.id] = obstacle.id
            hole = False

        if hole:
            id_remap[obstacle.id] += 100
            # print obstacle.id, ' remapped to ', id_remap[obstacle.id]

        # Use remapped ID
        current_id = id_remap[obstacle.id]

        while len(aux_poses) <= current_id:  # Loop to add aux poses for every agent
            aux_poses.append([])
            aux_frames.append([])

        while len(collection.tracklets) <= current_id:  # Loop to add tracker elements for every agent
            collection.tracklets.append([])

        if obstacle.occluded:  # Store inactive agents poses in auxiliary list
            aux_poses[current_id].append(pose)
            aux_frames[current_id].append(camera_frame)
            # print ('New inactive pose for ID %d' % obstacle.id)

        else:
            # print ('New pose received for ID %d' % obstacle.id)
            #collection.tracklets[obstacle.id].poses.append(pose)
            aux_poses[current_id].append(pose)
            aux_frames[current_id].append(camera_frame)

            if type(collection.tracklets[current_id]) is list:
                obs_tracklet = Tracklet(
                    object_type=obstacle.kind_name, l=obstacle.length, w=obstacle.width,
                    h=obstacle.height, first_frame=aux_frames[current_id][0])
                # collection.tracklets.insert(camera_frame, obs_tracklet)
                collection.tracklets[current_id] = obs_tracklet

            # except IndexError:
            #     print ('Obstacle with ID %d', obstacle.id, ' appended to list')
            #     obs_tracklet.poses.append(pose)
            # If there are poses in auxiliary list, append them before new active pose is inserted
            # if len(aux_poses) > obstacle.id and len(aux_poses[current_id]) > 0:
            for p in aux_poses[current_id]:
                collection.tracklets[current_id].poses.append(p)
            del(aux_poses[current_id][:])  # Remove all stored poses for current agent

    # end of obstacle loop
    last_frame = camera_frame
    print 'generator', last_frame

def obs_callback(obstacle_list):
    # Check if obstacles in msg and exit otherwise
    if len(obstacle_list.obstacles) != 1:
        return

    global obs_dict
    global last_frame
    for obstacle in obstacle_list.obstacles:
        timestamp = str(obstacle_list.header.stamp.to_nsec())[:-6]
        obs = {}
        # obs["timestamp"] = obstacle_list.header.stamp
        obs["tx"] = obstacle.location.x
        obs["ty"] = obstacle.location.y
        obs["tz"] = obstacle.location.z
        obs['rz'] = obstacle.yaw
        obs["l"] = obstacle.length
        obs["w"] = obstacle.width
        obs["h"] = obstacle.height
        obs["id"] = obstacle.id
        obs["kind_name"] = obstacle.kind_name
        obs["frame"] = int(frames[timestamp])
        obs_dict[timestamp] = obs
        if (int(frames[timestamp])-last_frame) != 1:
            print ('Hole of %d detected' % (int(frames[timestamp])-last_frame))
        last_frame = int(frames[timestamp])


def hack_callback():
    if len(camera_dict['timestamp']):
        camera_df = pd.DataFrame(data=camera_dict, columns=camera_cols)
        # Interpolate samples from all used sensors to camera frame timestamps
        camera_df['timestamp'] = pd.to_datetime(camera_df['timestamp'])
        camera_df.set_index(['timestamp'], inplace=True)
        camera_df.index.rename('index', inplace=True)
        camera_index_df = pd.DataFrame(index=camera_df.index)

        cap_rear_rtk_df = pd.DataFrame(data=cap_rear_rtk_dict, columns=rtk_cols)
        if not len(cap_rear_rtk_df.index):
            print('Error: No capture vehicle rear RTK entries exist.')
            return
        cap_front_rtk_df = pd.DataFrame(data=cap_front_rtk_dict, columns=rtk_cols)
        if not len(cap_front_rtk_df.index):
            print('Error: No capture vehicle front RTK entries exist.')
            return

        cap_rear_rtk_interp = interpolate_to_camera(camera_index_df, cap_rear_rtk_df, filter_cols=rtk_cols)
        cap_rear_rtk_interp_rec = cap_rear_rtk_interp.to_dict(orient='records')

        cap_front_rtk_interp = interpolate_to_camera(camera_index_df, cap_front_rtk_df, filter_cols=rtk_cols)
        cap_front_rtk_interp_rec = cap_front_rtk_interp.to_dict(orient='records')

        velo2_pos = defaultdict(list)
        gps_front = defaultdict(list)
        gps_rear = defaultdict(list)
        obs_pos = defaultdict(list)
        global rtk_cap_obs
        for i in range(0, len(cap_rear_rtk_interp_rec), 1):
            timestamp = str(int(cap_front_rtk_interp_rec[i]['timestamp']))[:-6]
            try:
                alt_pos, rot_z = get_velo_pos(cap_front_rtk_interp_rec[i], cap_rear_rtk_interp_rec[i])
                velo2_pos['tx'].append(alt_pos[0])
                velo2_pos['ty'].append(alt_pos[1])

                gps_front['tx'].append(cap_front_rtk_interp_rec[i]['tx'])
                gps_front['ty'].append(cap_front_rtk_interp_rec[i]['ty'])
                gps_front['tz'].append(cap_front_rtk_interp_rec[i]['tz'])

                gps_rear['tx'].append(cap_rear_rtk_interp_rec[i]['tx'])
                gps_rear['ty'].append(cap_rear_rtk_interp_rec[i]['ty'])
                gps_rear['tz'].append(cap_rear_rtk_interp_rec[i]['tz'])

                global_pos = get_global_obstacle_pos(cap_front_rtk_interp_rec[i],
                                                     cap_rear_rtk_interp_rec[i], obs_dict[timestamp])
                # print "Global pos ", global_pos
                obs_pos['tx'].append(global_pos['tx'])
                obs_pos['ty'].append(global_pos['ty'])
                obs_pos['tz'].append(global_pos['tz'])
                # local_pos = get_local_obstacle_pos(cap_front_rtk_interp_rec[i], cap_rear_rtk_interp_rec[i], global_pos)
                # print "Obstacle location ", obs_dict[timestamp]
                # print "Local pos ", local_pos

                # Fill RTK positions table
                rtk_cap_obs[frames[timestamp]] = [cap_front_rtk_interp_rec[i], cap_rear_rtk_interp_rec[i],
                                                  global_pos, obs_dict[timestamp]]

            except KeyError:
                # print timestamp, " not found"
                continue

        # Plot obstacle and front/rear rtk paths in absolute RTK ENU coords
        fig = plt.figure()
        plt.plot(
            obs_pos['tx'],
            obs_pos['ty'],
            'y-',
            velo2_pos['tx'],
            velo2_pos['ty'],
            'r-',
            gps_front['tx'],
            gps_front['ty'],
            'b-',
            gps_rear['tx'],
            gps_rear['ty'],
            'g-')

        # plot_path = os.path.expanduser('~/Desktop/')
        # if not os.path.exists(plot_path):
        #     os.makedirs(plot_path)
    	# plot_path = os.path.join(tracklet_path, "plot.png")
        # fig.savefig(plot_path)
        # plt.close(fig)

        # Create TrackletCollection with specified time shift
        timeshift = 10

        obstacle_list = []
        for i in range(0, len(cap_rear_rtk_interp_rec), 1):
            timestamp = str(int(cap_front_rtk_interp_rec[i]['timestamp']))[:-6]
        # try:
            # orig_pos = get_local_obstacle_pos(rtk_cap_obs[frames[timestamp]][0],
            #                                       rtk_cap_obs[frames[timestamp]][1],
            #                                       rtk_cap_obs[frames[timestamp]][2])
            # print "Original ", orig_pos
            #
            # print "Frame ", frames[timestamp]
            # print "Accediendo a ", int(frames[timestamp])+timeshift

            hack_obs = get_local_obstacle_pos(rtk_cap_obs[frames[timestamp]][0],
                                                  rtk_cap_obs[frames[timestamp]][1],
                                                  rtk_cap_obs[int(frames[timestamp])+timeshift][2])
            # Copy info for tracklet management
            hack_obs["rz"] = rtk_cap_obs[frames[timestamp]][3]["rz"]
            hack_obs["l"] = rtk_cap_obs[frames[timestamp]][3]["l"]
            hack_obs["w"] = rtk_cap_obs[frames[timestamp]][3]["w"]
            hack_obs["h"] = rtk_cap_obs[frames[timestamp]][3]["h"]
            hack_obs["id"] = rtk_cap_obs[frames[timestamp]][3]["id"]
            hack_obs["kind_name"] = rtk_cap_obs[frames[timestamp]][3]["kind_name"]
            hack_obs["frame"] = frames[timestamp]

            # print "Shifted", obstacle_pos
            obstacle_list.append(hack_obs)
        # except:
        #     continue


        print "Writing %d tracklets" % len(obstacle_list)
        # Write XML output
        for obstacle in obstacle_list:
            pose = {}
            pose['tx'] = obstacle['tx']
            pose['ty'] = obstacle['ty']
            pose['tz'] = obstacle['tz']
            pose['rx'] = 0
            pose['ry'] = 0
            pose['rz'] = obstacle["rz"]

            # TODO Deal with detection gaps
            try:
                collection.tracklets[obstacle["id"]].poses.append(pose)
                # print ('New pose received for ID', obstacle["id"])
            except IndexError:
                obs_tracklet = Tracklet(
                    object_type=obstacle["kind_name"], l=obstacle["l"], w=obstacle["w"],
                    h=obstacle["h"], first_frame=obstacle["frame"])
                obs_tracklet.poses.append(pose)
                # print obstacle
                collection.tracklets.insert(obstacle["id"], obs_tracklet)
                print ('Obstacle with ID', obstacle["id"], ' appended to list')
        write_tracklets_xml()

def write_tracklets_xml():
    global collection
    tracklet_path = os.path.os.path.expanduser('~/Desktop/Didi-Release-2/')
    if not os.path.exists(tracklet_path):
        os.makedirs(tracklet_path)
    tracklet_path = os.path.join(tracklet_path, "tracklets.xml")
    collection.write_xml(tracklet_path)
    print ('Tracklets file ready')

if __name__ == '__main__':
    rospy.init_node('tracklet_generator', anonymous=True)
    #rospy.Subscriber("interpolated_detections", ObstacleList, obstacle_callback)
    rospy.Subscriber("image_raw", Image, image_callback)
    # rospy.Subscriber("/objects/capture_vehicle/rear/gps/rtkfix", Odometry, rear_rtk_callback)
    # rospy.Subscriber("/objects/capture_vehicle/front/gps/rtkfix", Odometry, front_rtk_callback)
    rospy.Subscriber("interpolated_detections", ObstacleList, obstacle_callback)
    rospy.on_shutdown(write_tracklets_xml)
    # rospy.on_shutdown(hack_callback)

    print ("Hey man, keep calm, I'm preparing your tracklets...")
    rospy.spin()
