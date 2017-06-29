#include <didi_challenge/multi_smart_tracker.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

MultiSmartTracker::MultiSmartTracker(std::string algorithm, double score_threshold = 0.96,
    double distance_threshold = 100, int max_missings = 2, int min_detections =2){

    algorithm_ = algorithm;
    lastID_ = 0; // 0 means not assigned

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
    std::cout << "Detection ROI " << bb <<  std::endl;

    SmartTracker tracker = SmartTracker(algorithm_, image, obs_msg, min_detections_);
    if(tracker.is_active()) tracker.setid(++lastID_); // Set id if tracker is activated by first score

    trackers_.push_back(tracker); // Add to list
}

perception_msgs::ObstacleList MultiSmartTracker::get_obstacle_list(const cv::Mat &frame, double cell_size, int grid_dim){
    perception_msgs::ObstacleList obs_list;
    std::cout << "List size: " << trackers_.size() << std::endl;
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

    std::cout << "Published list size: " << obs_list.obstacles.size() << std::endl;
    return obs_list;
}

void MultiSmartTracker::update_prediction(cv::Mat frame, const perception_msgs::ObstacleList::ConstPtr& obs_msg){
    std::vector<int> to_remove;
    for (int i = 0; i < trackers_.size(); ++i){
        bool success = trackers_[i].predict(frame);
        if(!success) to_remove.push_back(i);
    }
    for (int i = 0; i < to_remove.size(); ++i){
         removeTracker(to_remove[i]);
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
    double current_threshold = 0.95;
    int unassigned_agents = tracker_map.size();
    int unassigned_detections = detection_map.size();
    while(unassigned_agents!=0 && current_threshold>0.4){ // Do Not consider under 0.6 scored detections
        std::cout << "Threshold: " << current_threshold << std::endl;

        for (std::map<int,bool>::iterator it=tracker_map.begin(); it!=tracker_map.end(); ++it){
            if(!tracker_map[it->first]) continue;
            cv::Point2d tracked_center;
            tracked_center.x = trackers_[it->first].get_roi().x + trackers_[it->first].get_roi().width/2;
            tracked_center.y = trackers_[it->first].get_roi().y + trackers_[it->first].get_roi().height/2;

            // Iterate through all detected objects to find the nearest to the current detection
            int best_index = -1;
            int best_distance = 999999;

            for (std::map<int,bool>::iterator it_d=detection_map.begin(); it_d!=detection_map.end(); ++it_d){

                if(!detection_map[it->first] || obs_msg->obstacles[it_d->first].score<current_threshold) continue;

                cv::Rect2d bb(obs_msg->obstacles[it_d->first].bbox.x_offset, obs_msg->obstacles[it_d->first].bbox.y_offset,
                    obs_msg->obstacles[it_d->first].bbox.width, obs_msg->obstacles[it_d->first].bbox.height);
                detections_.push_back(bb);

                cv::Point2d obs_center(bb.x + bb.width/2, bb.y + bb.height/2);

                double tocenterx = obs_center.x - 350;
                double tocentery = obs_center.y - 350;
                double distance_center = sqrt(tocenterx*tocenterx + tocentery*tocentery);
                if(distance_center * 0.1 < 1.5 ) continue;

                double dx = tracked_center.x - obs_center.x;
                double dy = tracked_center.y - obs_center.y;
                double current_distance = sqrt(dx*dx + dy*dy);

                if (current_distance < best_distance)
                {
                    best_distance = current_distance;
                    best_index = it_d->first;
                }
            }
            if (best_distance < distance_threshold_){
                std::cout << "Updating tracker with score: " << obs_msg->obstacles[best_index].score << std::endl;
                // Update the position of the old tracked object
                trackers_[it->first].update_roi(obs_msg->obstacles[best_index]);
                if(trackers_[it->first].is_active() && trackers_[it->first].getid()==0){  // Becomes active, set ID
                    lastID_++;
                    trackers_[it->first].setid(lastID_);
                }
                tracker_map[it->first] = false;
                detection_map[best_index]=false;
                unassigned_agents--;
                unassigned_detections--;
            }
        }
        current_threshold -=0.1;
    }

    // Detections not assigned to trackers become new Agents
    for (std::map<int,bool>::iterator it_d=detection_map.begin(); it_d!=detection_map.end(); ++it_d){
        // TODO Remove threshold. Tracker will not be not active but should be added
        if(detection_map[it_d->first] && obs_msg->obstacles[it_d->first].score > 0.9){
            // Check if detection overlaps with existing agents. If so, ignore them
            // Get detection center
            cv::Rect2d bb(obs_msg->obstacles[it_d->first].bbox.x_offset, obs_msg->obstacles[it_d->first].bbox.y_offset,
                obs_msg->obstacles[it_d->first].bbox.width, obs_msg->obstacles[it_d->first].bbox.height);
            cv::Point2d obs_center(bb.x + bb.width/2, bb.y + bb.height/2);


            bool should_add = true;
            // TODO REMOVE
            double tocenterx = obs_center.x - 350;
            double tocentery = obs_center.y - 350;
            double distance_center = sqrt(tocenterx*tocenterx + tocentery*tocentery);
            if(distance_center * 0.1 < 1.5 ) should_add = false;

            for (std::map<int,bool>::iterator it=tracker_map.begin(); should_add && it!=tracker_map.end(); ++it){
                // Get agent center
                cv::Point2d tracked_center;
                tracked_center.x = trackers_[it->first].get_roi().x + trackers_[it->first].get_roi().width/2;
                tracked_center.y = trackers_[it->first].get_roi().y + trackers_[it->first].get_roi().height/2;

                // Compute centers distance
                double dx = tracked_center.x - obs_center.x;
                double dy = tracked_center.y - obs_center.y;
                double current_distance = sqrt(dx*dx + dy*dy);
                if(current_distance * 0.05 < 2.5 ) should_add = false;
            }
            if(should_add) addTracker(frame, obs_msg->obstacles[it_d->first]);
        }
    }

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
        std::cout << "ID: " << tracker.getid() << " Score: " << tracker.get_score() << " Detection score: " << tracker.get_obstacle().score << std::endl;
        std::cout << "ROI " << tracker.get_roi() <<  std::endl;
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

    std::cout << "Tracked counts: " << tracked_counts << std::endl;
}
