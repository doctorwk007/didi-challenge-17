#include <didi_challenge/multi_smart_tracker.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

MultiSmartTracker::MultiSmartTracker(std::string algorithm, double cell_size, int grid_dim, double score_threshold = 0.96,
    double distance_threshold = 100, int max_missings = 2, int min_detections =2){

    algorithm_ = algorithm;
    lastID_ = 0; // 0 means not assigned

    cell_size_ = cell_size;
    grid_dim_ = grid_dim;

    score_threshold_ = score_threshold;
    distance_threshold_ = distance_threshold;
    max_missings_ = max_missings;
    min_detections_ = min_detections;
    tracked_counts = 0;
}

MultiSmartTracker::~MultiSmartTracker(){
    trackers_.clear();
}

void MultiSmartTracker::removeTracker(int index){
    trackers_.erase(trackers_.begin()+index);
}

void MultiSmartTracker::addTracker(const cv::Mat &image, perception_msgs::Obstacle obs_msg){
    cv::Rect2d bb(obs_msg.bbox.x_offset, obs_msg.bbox.y_offset, obs_msg.bbox.width, obs_msg.bbox.height);
    detections_.push_back(bb);
//    std::cout << "Detection ROI " << bb <<  std::endl;

    SmartTracker tracker = SmartTracker(algorithm_, image, obs_msg, min_detections_, max_missings_);
//    if(tracker.is_active()) tracker.setid(++lastID_); // Set id if tracker is activated by first score
    tracker.setid(++lastID_); // Set id to tracker  TODO CHANGED FROM PREVIOUS VERSION

    trackers_.push_back(tracker); // Add to list
}

perception_msgs::ObstacleList MultiSmartTracker::get_obstacle_list(const cv::Mat &frame, double cell_size, int grid_dim){
    perception_msgs::ObstacleList obs_list;
//    std::cout << "List size: " << trackers_.size() << std::endl;
    for (int i = 0; i < trackers_.size(); ++i){
//        if(trackers_[i].is_active()){
//            perception_msgs::Obstacle obs = trackers_[i].get_obstacle(frame, cell_size, grid_dim);
////            if(fabs(obs.location.x)>3.5 && fabs(obs.location.y)>1.5){ //Ego-car: width 1.86, height 4.93
////                if(obs.location.z>0.3 ) obs_list.obstacles.push_back(obs); // TODO Adjust threshold. Previously: 0.5
////            }
//
//            obs_list.obstacles.push_back(obs);
//        }

        // Now inactive agents are also sent
        perception_msgs::Obstacle obs = trackers_[i].get_obstacle(frame, cell_size, grid_dim);
        if(!trackers_[i].is_active()){
            obs.occluded = 1;
        }
        obs_list.obstacles.push_back(obs);
    }

//    std::cout << "Published list size: " << obs_list.obstacles.size() << std::endl;
    return obs_list;
}

void MultiSmartTracker::update_prediction(cv::Mat frame, const perception_msgs::ObstacleList::ConstPtr& obs_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud){
    std::vector<int> to_remove;
    for (int i = 0; i < trackers_.size(); ++i){
        cv::Point2d tracked_center;
        tracked_center.x = trackers_[i].get_roi().x + trackers_[i].get_roi().width/2;
        tracked_center.y = trackers_[i].get_roi().y + trackers_[i].get_roi().height/2;
        if(tracked_center.x > 0 && tracked_center.x < 700 && tracked_center.y > 0 && tracked_center.y<700){ // Avoid prediction of out of range radar agents
            bool success = trackers_[i].predict(frame);
            if(!success) to_remove.push_back(i);
        }
    }

    if(cell_size_ == 0.05) {//ONLY FOR PEDESTRIANS?
        for (int i = 0; i < to_remove.size(); ++i){
            removeTracker(to_remove[i]);
        }
    }

    // Update tracked objects with the new detection and add new ones
    detections_.clear(); // Reset detection vector for this frame
    // int i = 0;
    // double current_score = obs_msg->obstacles[i].score;

    std::map<int,bool> tracker_map;
    for (int j = 0; j < trackers_.size(); ++j){
        tracker_map[j]=true;
    }
    std::map<int,bool> detection_map;
    for (int j = 0; j < obs_msg->obstacles.size(); ++j){
        detection_map[j]=true;
    }
    std::map<int,bool> radar_map;
    for (int j = 0; j < radar_cloud->points.size(); ++j){
        radar_map[j]=true;
    }

    double current_threshold = 0.95;
    int unassigned_agents = tracker_map.size();
    int unassigned_detections = detection_map.size();
    while(unassigned_agents!=0 && current_threshold>0.4){ // Do Not consider under 0.6 scored detections
//        std::cout << "Threshold: " << current_threshold << std::endl;

        for (std::map<int,bool>::iterator it=tracker_map.begin(); it!=tracker_map.end(); ++it){
            if(!tracker_map[it->first]) continue;
            cv::Point2d tracked_center;
            tracked_center.x = trackers_[it->first].get_roi().x + trackers_[it->first].get_roi().width/2;
            tracked_center.y = trackers_[it->first].get_roi().y + trackers_[it->first].get_roi().height/2;

            // Iterate through all detected objects to find the nearest to the current detection
            int best_index = -1;
            double best_distance = 999999;
            double capture_distance = 999999;
            bool match_radar = false;
            for (std::map<int,bool>::iterator it_d=detection_map.begin(); it_d!=detection_map.end(); ++it_d){

                if(!detection_map[it_d->first] || obs_msg->obstacles[it_d->first].score<current_threshold) continue;

                cv::Rect2d bb(obs_msg->obstacles[it_d->first].bbox.x_offset, obs_msg->obstacles[it_d->first].bbox.y_offset,
                    obs_msg->obstacles[it_d->first].bbox.width, obs_msg->obstacles[it_d->first].bbox.height);
                detections_.push_back(bb);

                cv::Point2d obs_center(bb.x + bb.width/2, bb.y + bb.height/2);

                double tocenterx = obs_center.x - grid_dim_/2/cell_size_;
                double tocentery = obs_center.y - grid_dim_/2/cell_size_;
                double distance_center = sqrt(tocenterx*tocenterx + tocentery*tocentery);
                if(distance_center * cell_size_ < 1.5 ) continue;

                double dx = tracked_center.x - obs_center.x;
                double dy = tracked_center.y - obs_center.y;
                double current_distance = sqrt(dx*dx + dy*dy);

                if (current_distance < best_distance)
                {
                    best_distance = current_distance;
                    best_index = it_d->first;
                }
            }
//
//            if(std::find(to_remove.begin(), to_remove.end(), it->first) == to_remove.end() && best_distance==999999){ // CHeck for radar if image does not work
//                cout << "Prediction and detection failed, checking now with radars: " << radar_map.size() << endl;
//                for (std::map<int,bool>::iterator it_r=radar_map.begin(); it_r!=radar_map.end(); ++it_r){
//
//    //                std::cout <<" radar!" << radar_map[it_r->first] << std::endl;
//                    if(!radar_map[it_r->first]) continue; // Skip if assigned
//                    cv::Point2d radar2d;
//                    radar2d.x = -(radar_cloud->points[it_r->first].y/cell_size_)+grid_dim_/2/cell_size_;
//                    radar2d.y = -(radar_cloud->points[it_r->first].x/cell_size_)+grid_dim_/2/cell_size_;
//                    double dx = fabs(tracked_center.x - radar2d.x);
//                    double dy = fabs(tracked_center.y - radar2d.y);
//                    double current_distance = sqrt(dx*dx + dy*dy);
//
//                    double cap_dx = fabs(grid_dim_/2/cell_size_ - radar2d.x);
//                    double cap_dy = fabs(grid_dim_/2/cell_size_ - radar2d.y);
//                    double capture_dis =sqrt(cap_dx*cap_dx + cap_dy*cap_dy);
//
//                    cout << "Distance to tracker: "<< current_distance <<" Radar beam at  " << capture_dis << endl;
//                    cout << "Point " << radar2d << endl;
//                    cout << trackers_[it->first].get_roi() << endl;
//                    if (current_distance < best_distance && current_distance * cell_size_ < 3){ // TODO BEFORE 2.5
//                        if(capture_dis<capture_distance){
//                            best_distance = current_distance;
//                            best_index = it_r->first;
//                            match_radar = true;
//                            capture_distance = capture_dis;
//                        }
//                    }
//                }
//            }


            if (best_distance < distance_threshold_){
                if(!match_radar){
//                    std::cout << "Updating tracker with detection: " << obs_msg->obstacles[best_index].score << std::endl;
                    // Update the position of the old tracked object
                    std::cout << "Updating tracker "<< trackers_[it->first].getid() << ". Detection score: " << obs_msg->obstacles[best_index].score
                        << ". Prev score: " << trackers_[it->first].get_score() <<  " -> ";
                    trackers_[it->first].update_roi(obs_msg->obstacles[best_index]);
                    if(trackers_[it->first].is_active() && trackers_[it->first].getid()==0){  // Becomes active, set ID
                        lastID_++;
                        trackers_[it->first].setid(lastID_);
                    }
                    std::cout << trackers_[it->first].get_score() << std::endl;
                    tracker_map[it->first] = false;
                    detection_map[best_index]=false;
                    unassigned_agents--;
                    unassigned_detections--;
                }else{
                    std::cout << "Updating tracker "<< trackers_[it->first].getid() << " with radar at "<< best_distance << std::endl;
                    // Update the position of the old tracked object
                    cv::Point2d tracked_center;
                    tracked_center.x = trackers_[it->first].get_roi().x + trackers_[it->first].get_roi().width/2;
                    tracked_center.y = trackers_[it->first].get_roi().y + trackers_[it->first].get_roi().height/2;

                    cout << "Prev center " << tracked_center << endl;
                    trackers_[it->first].update_with_radar(radar_cloud->points[best_index]);

                    tracked_center.x = trackers_[it->first].get_roi().x + trackers_[it->first].get_roi().width/2;
                    tracked_center.y = trackers_[it->first].get_roi().y + trackers_[it->first].get_roi().height/2;

                    cout << "New center " << tracked_center << endl;

                    if(trackers_[it->first].is_active() && trackers_[it->first].getid()==0){  // Becomes active, set ID
                        lastID_++;
                        trackers_[it->first].setid(lastID_);
                    }
                    tracker_map[it->first] = false;
                    radar_map[best_index]=false;
                    unassigned_agents--;
                }
            }
        }
        current_threshold -=0.1;
    }

    // Detections not assigned to trackers become new Agents
    for (std::map<int,bool>::iterator it_d=detection_map.begin(); it_d!=detection_map.end(); ++it_d){
        // TODO Remove threshold. Tracker will not be not active but should be added
        if(detection_map[it_d->first] && obs_msg->obstacles[it_d->first].score > 0.5){ // TODO 0.9
            // Check if detection overlaps with existing agents. If so, ignore them
            // Get detection center
            cv::Rect2d bb(obs_msg->obstacles[it_d->first].bbox.x_offset, obs_msg->obstacles[it_d->first].bbox.y_offset,
                obs_msg->obstacles[it_d->first].bbox.width, obs_msg->obstacles[it_d->first].bbox.height);
            cv::Point2d obs_center(bb.x + bb.width/2, bb.y + bb.height/2);

            detections_.push_back(bb);

            bool should_add = true;
            // REMOVE CAPTURE VEHICLE
            double tocenterx = obs_center.x - grid_dim_/2/cell_size_;
            double tocentery = obs_center.y - grid_dim_/2/cell_size_;
            double distance_center = sqrt(tocenterx*tocenterx + tocentery*tocentery);
            if(distance_center * cell_size_ < 1.5 ) should_add = false; // CHECK PARAMETER PED/VEHICLE

            for (std::map<int,bool>::iterator it=tracker_map.begin(); should_add && it!=tracker_map.end(); ++it){
                // Get agent center
                cv::Point2d tracked_center;
                tracked_center.x = trackers_[it->first].get_roi().x + trackers_[it->first].get_roi().width/2;
                tracked_center.y = trackers_[it->first].get_roi().y + trackers_[it->first].get_roi().height/2;

                // Compute centers distance
                double dx = tracked_center.x - obs_center.x;
                double dy = tracked_center.y - obs_center.y;
                double current_distance = sqrt(dx*dx + dy*dy);
                if(current_distance * cell_size_ < 4 ) should_add = false; // CHECK PARAMETER PED/VEHICLE todo check  2.5 antes
            }

            if(should_add){
                bool weak_detection_and_radar = false;
                for (std::map<int,bool>::iterator it_r=radar_map.begin(); it_r!=radar_map.end(); ++it_r){
   //
                    if(!radar_map[it_r->first]) continue; // Skip if assigned
                    cv::Point2d radar2d;
                    radar2d.x = -(radar_cloud->points[it_r->first].y/cell_size_)+grid_dim_/2/cell_size_;
                    radar2d.y = -(radar_cloud->points[it_r->first].x/cell_size_)+grid_dim_/2/cell_size_;
                    double dx = fabs(obs_center.x - radar2d.x);
                    double dy = fabs(obs_center.y - radar2d.y);
                    double current_distance = sqrt(dx*dx + dy*dy);

                    if (current_distance * cell_size_ < 2){
                        weak_detection_and_radar = true;
                    }
                }
                cout << "Score " << obs_msg->obstacles[it_d->first].score << endl;
                perception_msgs::Obstacle o = obs_msg->obstacles[it_d->first];
                if(weak_detection_and_radar) {
                    o.score = std::max(1., o.score+0.2);
                    cout << "\nWeak detection activated by radar!!\n";
                }
                addTracker(frame, o);
            }
        }
    }

    // Detections from radar not assigned to any agent are ignored TODO REVIEW

    for (std::map<int,bool>::iterator it=tracker_map.begin(); it!=tracker_map.end(); ++it){
        // Decrease tracking score
        trackers_[it->first].set_missing();
    }

    // Check trackers_ to remove
    for (int i = 0; i < trackers_.size(); ++i){
        // if(trackers_[i].get_missings()>max_missings_) removeTracker(i);
        if(trackers_[i].get_score()<0.1) removeTracker(i);
    }
}

void MultiSmartTracker::draw_detections(cv::Mat& image)
{
    // std::cout << "Total active trackers: " << active_counter << std::endl;
    // std::cout << "Total detections: " << detections_.size() << std::endl;
    //Then draw the new detections
    for (auto obs : detections_)
    {
        cv::rectangle(image, obs, RED, 2, 1);
    }

    int active_counter = 0;
    // std::cout << "-------------" << std::endl;
    // Draw first the tracked objects
    for (auto tracker : trackers_)
    {
//        std::cout << "ID: " << tracker.getid() << " Score: " << tracker.get_score() << " Detection score: " << tracker.get_obstacle().score << std::endl;
//        std::cout << "ROI " << tracker.get_roi() <<  std::endl;
        std::string text = std::to_string(tracker.getid());
        int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontScale = 0.6;
        int thickness = 1;
        cv::Point textOrg(tracker.get_roi().x+tracker.get_roi().width, tracker.get_roi().y);
        if(tracker.is_active()){
            active_counter++;
            tracked_counts++;
            cv::rectangle(image, tracker.get_roi(), GREEN, 2, 1);
            cv::putText(image, text, textOrg, fontFace, fontScale, GREEN, thickness,8);
        }else{
            cv::rectangle(image, tracker.get_roi(), PURPLE, 2, 1);
            cv::putText(image, text, textOrg, fontFace, fontScale, PURPLE, thickness,8);
        }
    }

//    std::cout << "Tracked counts: " << tracked_counts << std::endl;
}
