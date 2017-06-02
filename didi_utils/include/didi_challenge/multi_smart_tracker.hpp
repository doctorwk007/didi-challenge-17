#ifndef MULTI_SMART_TRACKER_HPP
#define MULTI_SMART_TRACKER_HPP

#include <didi_challenge/smart_tracker.hpp>

#include <perception_msgs/ObstacleList.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>

const cv::Scalar BLUE(255,0,0);
const cv::Scalar GREEN(0, 255,0);
const cv::Scalar RED(0,0,255);
const cv::Scalar PURPLE(128,0,128);
const std::string DRAW_WINDOW = "Detections";

class MultiSmartTracker
{
public:
    MultiSmartTracker(std::string algorithm, double score_threshold,
        double distance_threshold, int max_missings, int min_detections);
    ~MultiSmartTracker();
    void update_prediction(cv::Mat frame, const perception_msgs::ObstacleList::ConstPtr& obs_msg);
    void draw_detections(cv::Mat& image);
    void removeTracker(int position);
    void addTracker(const cv::Mat &image, perception_msgs::Obstacle obs_msg);
    perception_msgs::ObstacleList get_obstacle_list(const cv::Mat &frame, double cell_size, int grid_dim);

    std::vector<SmartTracker> trackers_;
    std::vector<cv::Rect2d> detections_;
private:
    std::string algorithm_; // Tracking algorithm
    int lastID_;

    double score_threshold_;
    double distance_threshold_;
    int max_missings_;
    int min_detections_;
    int tracked_counts;
};

#endif
