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
    max_missings_ = 5;
}

SmartTracker::SmartTracker(std::string tracking_algorithm, cv::Mat init_frame, perception_msgs::Obstacle init_object, double min_detections, int max_missings)
{
    tracker_ptr_ = cv::Tracker::create(tracking_algorithm);
    num_missings_ = 0;
    num_detected_ = 0;
    min_detections_ = min_detections;
    perfect_score_ = 3.;
    tracking_score_ = 0.;
    last_nonzero_ = 0;
    max_missings_ = max_missings;

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
    if(new_detection.score >0.85){
        tracking_score_ *= pow(1+new_detection.score-0.75, num_detected_); // CHANGED TO FORCE SCORE TO RISE WHEN DETECTION
        if(tracking_score_ > 0.9) tracking_score_ = std::max(tracking_score_, 1.); // Activate if high confidence
    }else{
        if(num_detected_>2){
            tracking_score_ = std::max(0.85, tracking_score_);
        }
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


double angle_between(Eigen::Vector3d axis1, Eigen::Vector3d axis2){
    axis1.normalize();
    axis2.normalize();
    double rad;
    if(axis1==axis2){
        rad = 0;
    }else{
        double cos = axis1.dot(axis2);
        rad = acos(cos);
        if(axis1[1]<0)
            rad=-rad;
    }
    return rad;
}

void SmartTracker::update_with_radar(pcl::PointXYZ new_detection)
{
    double cell_size = 0.1; // TODO PARAMETER
    int map_size = 700;
    // Get 2D coords for the 3D radar point
    cv::Point2d radar2d;
    radar2d.x = -(new_detection.y/cell_size)+map_size/2;
    radar2d.y = -(new_detection.x/cell_size)+map_size/2;
    // Get previous center
    cv::Point2d tracked_center;
    tracked_center.x = obs_.bbox.x_offset + obs_.bbox.width/2;
    tracked_center.y = obs_.bbox.y_offset + obs_.bbox.height/2;

    double dx = fabs(tracked_center.x - radar2d.x);
    double dy = fabs(tracked_center.y - radar2d.y);
    double current_distance = sqrt(dx*dx + dy*dy);

//    cout << "Previous bbox " << obs_.bbox << endl;
//    if(current_distance>25) { // TODO A PELO Better check if radar point is inside bbox with margin ?
//
//        cout << "New yaw and ROI update" << endl;
//        // Estimate yaw angle using agent's previous center
//        static Eigen::Vector3d front_direction(1.0,0.0,0.0);
//        Eigen::Vector3d car_vector(radar2d.x-tracked_center.x,radar2d.y-tracked_center.y,0);
//        double yaw=angle_between(car_vector,front_direction);
//        double syaw = sin(yaw);
//        double cyaw = cos(yaw);
//
//        // Compute new corners
//        cv::Point2d new_tl;
//        new_tl.x = cyaw * (obs_.bbox.x_offset-radar2d.x) - syaw * (obs_.bbox.y_offset-radar2d.y) + radar2d.x;
//        new_tl.y = syaw * (obs_.bbox.x_offset-radar2d.x) + cyaw * (obs_.bbox.y_offset-radar2d.y) + radar2d.y;
//
//        cv::Point2d new_br;
//        new_br.x = cyaw * (obs_.bbox.x_offset+obs_.bbox.width-radar2d.x) - syaw * (obs_.bbox.y_offset+obs_.bbox.height-radar2d.y) + radar2d.x;
//        new_br.y = syaw * (obs_.bbox.x_offset+obs_.bbox.width-radar2d.x) + cyaw * (obs_.bbox.y_offset+obs_.bbox.height-radar2d.y) + radar2d.y;
//
//        if(new_br.x<new_tl.x){
//            double auxx = new_br.x;
//            new_br.x = new_tl.x;
//            new_tl.x = auxx;
//        }
//        if(new_br.y<new_tl.y){
//            double auxy = new_br.y;
//            new_br.y = new_tl.y;
//            new_tl.y = auxy;
//        }
//        // Update obstacle bbox
//        obs_.bbox.x_offset = new_tl.x;
//        obs_.bbox.y_offset = new_tl.y;
//        obs_.bbox.width = new_br.x-new_tl.x;
//        obs_.bbox.height = new_br.y-new_tl.y;
//        obs_.alpha = yaw;
//        obs_.yaw = yaw;
//        cout << "yaw : " << yaw << endl;
//    }else{
//        cout << "Not distance for yaw" << endl;
        obs_.bbox.y_offset = radar2d.y-obs_.bbox.height;
        if(radar2d.x<map_size/2 - 20){
            obs_.bbox.x_offset = radar2d.x-obs_.bbox.width;
        }else if (radar2d.x>map_size/2 + 20){
            obs_.bbox.x_offset = radar2d.x;
        }else{
            obs_.bbox.x_offset = radar2d.x-obs_.bbox.width/2;
        }
//    }
    num_detected_++;
    num_missings_= 0; // Reset counter

//    cout << "bbox " << obs_.bbox << endl;

    // Detection found, Increase tracking score
    tracking_score_ *= pow(1+0.95-0.85, num_detected_); // TODO CHECK Score manually set
    if(tracking_score_>2) tracking_score_ = 2;
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
    return tracking_score_ > 0.85; // TODO Param One detection with  90% activates the tracker CHANGED FROM 0.90
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
//    tracking_score_ *= pow(0.98, num_missings_); // Original version
    tracking_score_ *= pow(0.98, num_missings_/max_missings_); // TODO CHANGED
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
