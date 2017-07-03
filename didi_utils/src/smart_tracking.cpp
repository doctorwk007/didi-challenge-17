#include <iostream>
#include <memory>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <perception_msgs/ObstacleList.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <didi_challenge/multi_smart_tracker.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <radar_driver/RadarTracks.h>
#include <radar_driver/Track.h>
using namespace std;

ros::Publisher obst_list_pub_, radar2velo_pub;
image_transport::Publisher tracking_pub;

double score_threshold;
double distance_threshold;
double cell_size;
int grid_dim;
int max_missings, min_detections;

unique_ptr<MultiSmartTracker> tracker;
tf::StampedTransform r2v_transform;
int ncalls = 0;

void detection_callback(const sensor_msgs::Image::ConstPtr& img_msg, const perception_msgs::ObstacleList::ConstPtr& obs_msg, const sensor_msgs::PointCloud2::ConstPtr & radar_msg)
{
    // RADAR PROCESSING
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud(new pcl::PointCloud<pcl::PointXYZ>); // After transformation
    fromROSMsg(*radar_msg, *radar_cloud);

    pcl_ros::transformPointCloud (*radar_cloud, *velo_cloud, r2v_transform); // velo_cloud -> Radar points in velodyne coords
    velo_cloud->header.frame_id = "/velodyne";

    sensor_msgs::PointCloud2 radar2velo;
    pcl::toROSMsg(*velo_cloud, radar2velo);
    radar2velo.header.stamp = radar_msg->header.stamp;
    radar2velo_pub.publish(radar2velo);

//    // TRACKING
    cv::Mat frame = cv_bridge::toCvCopy(img_msg)->image;
    tracker->update_prediction(frame, obs_msg, velo_cloud);
    // Draw the detections
    tracker->draw_detections(frame);

    cv_bridge::CvImage cv_bird_view;
    cv_bird_view.header = obs_msg->header;
    cv_bird_view.encoding = "bgr8";
    cv_bird_view.image = frame;
    tracking_pub.publish(cv_bird_view.toImageMsg());

    // Publish ObstacleList to ros
    perception_msgs::ObstacleList obs_list = tracker->get_obstacle_list(frame, cell_size, grid_dim);
    if(obs_list.obstacles.size()>0){ // Send msg only if there are obstacles
        obs_list.header = obs_msg->header;
        obst_list_pub_.publish(obs_list);
    }
    cout << "Num callbacks " << ++ncalls << endl;
}

void original_callback(const sensor_msgs::Image::ConstPtr& img_msg, const perception_msgs::ObstacleList::ConstPtr& obs_msg){

    cout << "Num callbacks " << ++ncalls << endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "smart_tracking");
    ros::NodeHandle public_nh, private_nh("~");
    image_transport::ImageTransport it(public_nh);

    tracking_pub = it.advertise("tracking", 1);

    string image_topic;
    private_nh.param<string>("image_topic", image_topic, "/bird_view");
    string obstacles_topic;
    private_nh.param<string>("obstacles_topic", obstacles_topic, "/cnn/filtered_detection");
    string output_topic;
    private_nh.param<string>("output_topic", output_topic, "filtered_cloud");
    string tracking_algorithm;
    private_nh.param<string>("tracking_algorithm", tracking_algorithm, "MEDIANFLOW");

    private_nh.param("score_threshold", score_threshold, 0.96);
    private_nh.param("distance_threshold", distance_threshold, 100.0);
    private_nh.param("max_missings", max_missings, 3);
    private_nh.param("min_detections", min_detections, 2);


    public_nh.param("/birdview/cell_size", cell_size, 0.1);
    public_nh.param("/birdview/grid_dim", grid_dim, 70);

    cout << "Using a grid of " << grid_dim << " square meters with resolution "<<cell_size << endl;
    cout << "Max missings " << max_missings<< ". Min detections "<< min_detections<< endl;

    obst_list_pub_ = private_nh.advertise<perception_msgs::ObstacleList> ("obstacles", 1);
    // Init the MultiTracker pointer
    tracker = unique_ptr<MultiSmartTracker>(new MultiSmartTracker(tracking_algorithm, cell_size, grid_dim, score_threshold, distance_threshold, max_missings, min_detections));

    // Init the synchronized subscribers
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    image_sub.subscribe(public_nh, image_topic, 1000);
    message_filters::Subscriber<perception_msgs::ObstacleList> detection_sub;
    detection_sub.subscribe(public_nh, obstacles_topic, 1000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub(public_nh, "/radar/points", 1000); 


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, perception_msgs::ObstacleList, sensor_msgs::PointCloud2> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub, detection_sub, radar_sub);
    sync.registerCallback(boost::bind(&detection_callback, _1, _2, _3));

//    message_filters::TimeSynchronizer<sensor_msgs::Image, perception_msgs::ObstacleList> sync(image_sub, detection_sub, 100);
//    sync.registerCallback(boost::bind(&original_callback, _1, _2));

    tf::TransformListener listener;
    try{
      listener.waitForTransform("/velodyne", "/radar", ros::Time(0), ros::Duration(20.0));
      listener.lookupTransform ("/velodyne", "/radar", ros::Time(0), r2v_transform);
    }catch (tf::TransformException& ex) {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
//      return -1;
    }


    radar2velo_pub = public_nh.advertise<sensor_msgs::PointCloud2> ("radar_points", 1);

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        cv::waitKey(1);
        ros::spinOnce();
        rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
}
