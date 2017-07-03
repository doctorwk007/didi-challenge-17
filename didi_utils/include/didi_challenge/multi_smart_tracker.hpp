#ifndef MULTI_SMART_TRACKER_HPP
#define MULTI_SMART_TRACKER_HPP

#include <didi_challenge/smart_tracker.hpp>

#include <perception_msgs/ObstacleList.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

const cv::Scalar BLUE(255,0,0);
const cv::Scalar GREEN(0, 255,0);
const cv::Scalar RED(0,0,255);
const cv::Scalar PURPLE(128,0,128);
const cv::Scalar YELLOW(0,255,255);

class MultiSmartTracker
{
public:
    MultiSmartTracker(std::string algorithm, double cell_size, int grid_dim,
        double score_threshold, double distance_threshold, int max_missings, int min_detections);
    ~MultiSmartTracker();
    void update_prediction(cv::Mat frame, const perception_msgs::ObstacleList::ConstPtr& obs_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud);
    void draw_detections(cv::Mat& image);
    void removeTracker(int position);
    void reset(){ trackers_.clear();}
    int addTracker(const cv::Mat &image, perception_msgs::Obstacle obs_msg);
    perception_msgs::ObstacleList get_obstacle_list(const cv::Mat &frame, double cell_size, int grid_dim);

    void prepare_trackers(){
        for(auto& tracker : trackers_){
            tracker.get_ready();
        }
    cout << "All trackers ready" << endl;
    }

    std::vector<SmartTracker> trackers_;
    std::vector<cv::Rect2d> detections_;
    std::vector<cv::Point2d> tracks_;
private:
    bool isRadarInROI(cv::Point2d p, cv::Rect2d roi);
    std::string algorithm_; // Tracking algorithm
    int lastID_;

    double cell_size_;
    double grid_dim_;

    double score_threshold_;
    double distance_threshold_;
    int max_missings_;
    int min_detections_;
    int tracked_counts;
};

#endif
