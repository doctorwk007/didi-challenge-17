#ifndef SMART_TRACKER_HPP
#define SMART_TRACKER_HPP

#include <perception_msgs/Obstacle.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>


class SmartTracker
{
public:
    SmartTracker(std::string tracking_algorithm);
    SmartTracker(std::string tracking_algorithm, cv::Mat init_frame, perception_msgs::Obstacle init_object, double perfect_score);
    ~SmartTracker();

    void init(cv::Mat init_frame, perception_msgs::Obstacle init_object);
    bool predict(cv::Mat& new_frame);
    void update_roi(perception_msgs::Obstacle new_detection);
    void setid(int id) {obs_.id = id;}
    void set_missing(); // Increase missing counter by 1

    // Get the number of consecutive detection updates / missing frames
    int get_detections();
    int get_missings();
    bool is_active();
    int getid() {return obs_.id;}
    double get_score(){return tracking_score_;}
    cv::Rect2d get_roi();
    perception_msgs::Obstacle get_obstacle(){return obs_;}
    perception_msgs::Obstacle get_obstacle(const cv::Mat &frame, double cell_size, int grid_dim);

private:
    cv::Mat * current_frame_;
    cv::Ptr<cv::Tracker> tracker_ptr_;
    perception_msgs::Obstacle obs_;
    int num_detected_;
    int num_missings_;
    int min_detections_;
    double perfect_score_;
    double tracking_score_;

    int last_nonzero_;
};




#endif
