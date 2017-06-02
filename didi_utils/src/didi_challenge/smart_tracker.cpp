#include <didi_challenge/smart_tracker.hpp>

SmartTracker::SmartTracker(std::string tracking_algorithm)
{
    tracker_ptr_ = cv::Tracker::create(tracking_algorithm);

    num_missings_ = 0;
    num_detected_ = 0;
    min_detections_ = 2;
    perfect_score_ = 3.;
    tracking_score_ = 0.;
    last_nonzero_ = 0;
}

SmartTracker::SmartTracker(std::string tracking_algorithm, cv::Mat init_frame, perception_msgs::Obstacle init_object, double perfect_score)
{
    tracker_ptr_ = cv::Tracker::create(tracking_algorithm);
    num_missings_ = 0;
    num_detected_ = 0;
    min_detections_ = 2;
    perfect_score_ = perfect_score;
    tracking_score_ = 0.;
    last_nonzero_ = 0;

    init(init_frame, init_object);
}

SmartTracker::~SmartTracker(){
    tracker_ptr_.release();
}

void SmartTracker::init(cv::Mat init_frame, perception_msgs::Obstacle init_object)
{
    cv::Rect2d bb(init_object.bbox.x_offset, init_object.bbox.y_offset, init_object.bbox.width, init_object.bbox.height);
    tracker_ptr_->init(init_frame, bb);
    obs_ = init_object;
    tracking_score_ = obs_.score; // Initializa score
}


bool SmartTracker::predict(cv::Mat& new_frame)
{
    current_frame_ = &new_frame;
    cv::Rect2d bb;
    bool success = tracker_ptr_->update(new_frame, bb);
    if(success){
        // Set new ROI values
        // std::cout << "ID " << obs_.id << " Exitaso: " << bb << std::endl;
        obs_.bbox.x_offset = bb.x;
        obs_.bbox.y_offset = bb.y;
        obs_.bbox.width = bb.width;
        obs_.bbox.height = bb.height;
    }
    return success;
}


void SmartTracker::update_roi(perception_msgs::Obstacle new_detection)
{
    // Update obstacle values
    obs_.bbox.x_offset = new_detection.bbox.x_offset;
    obs_.bbox.y_offset = new_detection.bbox.y_offset;
    obs_.bbox.width = new_detection.bbox.width;
    obs_.bbox.height = new_detection.bbox.height;
    obs_.alpha = new_detection.alpha;
    obs_.yaw = new_detection.yaw;
    obs_.velocity = new_detection.velocity;
    num_detected_++;
    num_missings_= 0; // Reset counter

    // Detection found, Increase tracking score
    if(obs_.score >0.85){
        tracking_score_ *= pow(1+obs_.score-0.85, num_detected_);
    }
    if(tracking_score_>2) tracking_score_ = 2;

    // if(tracking_score_>10) tracking_score_ = 10;

    // // Get max height in ROI and set Z coord to height/2
    // cv::Rect2d boundingBox(new_detection.bbox.x_offset, new_detection.bbox.y_offset, new_detection.bbox.width, new_detection.bbox.height);
    // if(boundingBox.x<0) boundingBox.x = 0;
    // if(boundingBox.y<0) boundingBox.y = 0;
    // if(boundingBox.x+boundingBox.width>current_frame_->cols) boundingBox.x = current_frame_->cols - boundingBox.width;
    // if(boundingBox.y+boundingBox.height>current_frame_->rows) boundingBox.y = current_frame_->rows - boundingBox.height;
    //
    // cv::Mat roi(*current_frame_, boundingBox);
    // cv::Mat bgr[3];
    // cv::split(roi, bgr);

    // Update nonzero variable
    // last_nonzero_ = cv::countNonZero(bgr[0]);
    // std::cout << " nonzero " << last_nonzero_ << std::endl;
}


perception_msgs::Obstacle SmartTracker::get_obstacle(const cv::Mat &frame, double cell_size, int grid_dim){
    int map_size = (int) (grid_dim/cell_size);
    // Update location values and return
    cv::Point2d tracked_center;
    tracked_center.x = obs_.bbox.x_offset + obs_.bbox.width/2;
    tracked_center.y = obs_.bbox.y_offset + obs_.bbox.height/2;

    // BBox center to real relative coords
    obs_.location.x = -(tracked_center.y-map_size/2)*cell_size;
    obs_.location.y = -(tracked_center.x-map_size/2)*cell_size;

    // Get max height in ROI and set Z coord to height/2
    cv::Rect2d boundingBox(obs_.bbox.x_offset, obs_.bbox.y_offset, obs_.bbox.width, obs_.bbox.height);
    if(boundingBox.x<0) boundingBox.x = 0;
    if(boundingBox.y<0) boundingBox.y = 0;
    if(boundingBox.x+boundingBox.width>frame.cols) boundingBox.x = frame.cols - boundingBox.width;
    if(boundingBox.y+boundingBox.height>frame.rows) boundingBox.y = frame.rows - boundingBox.height;

    cv::Mat roi = frame(boundingBox);
    cv::Mat bgr[3]; // Height, density, intensity
    cv::split(roi, bgr);
    double minVal; double maxVal;
    cv::minMaxLoc(bgr[0], &minVal, &maxVal);
    obs_.height =  (maxVal/255)*3.0;
    obs_.location.z = obs_.height/2;

    if(obs_.bbox.width < obs_.bbox.height){
        obs_.width = obs_.bbox.width * cell_size;
        obs_.length = obs_.bbox.height * cell_size;
    }else{
        obs_.width = obs_.bbox.height * cell_size;
        obs_.length = obs_.bbox.width * cell_size;
    }
    return obs_;
}

bool SmartTracker::is_active(){
    // return num_detected_ > (min_detections_-1);
    return tracking_score_ > 0.90; // TODO Param One detection with score 90% activates the tracker
}

int SmartTracker::get_detections()
{
    return num_detected_;
}

void SmartTracker::set_missing()
{
    ++num_missings_;
    num_detected_ = 0; // Reset counter
    // Get max height in ROI and set Z coord to height/2
    cv::Rect2d boundingBox(obs_.bbox.x_offset, obs_.bbox.y_offset, obs_.bbox.width, obs_.bbox.height);
    if(boundingBox.x<0) boundingBox.x = 0;
    if(boundingBox.y<0) boundingBox.y = 0;
    if(boundingBox.x+boundingBox.width>current_frame_->cols) boundingBox.x = current_frame_->cols - boundingBox.width;
    if(boundingBox.y+boundingBox.height>current_frame_->rows) boundingBox.y = current_frame_->rows - boundingBox.height;

    cv::Mat roi(*current_frame_, boundingBox);
    cv::Mat bgr[3];
    cv::split(roi, bgr);

    // Decrease nonzero variable
    // tracking_score_ -= (abs(last_nonzero_-cv::countNonZero(bgr[0])))/last_nonzero_ * perfect_score_; //TODO FIX
    tracking_score_ *= pow(0.98, num_missings_);
}

int SmartTracker::get_missings()
{
    return num_missings_;
}

cv::Rect2d SmartTracker::get_roi()
{
    cv::Rect2d bb(obs_.bbox.x_offset, obs_.bbox.y_offset, obs_.bbox.width, obs_.bbox.height);
    return bb;
}
