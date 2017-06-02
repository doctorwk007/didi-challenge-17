#ifndef OBSTACLE_UTILS_HPP
#define OBSTACLE_UTILS_HPP

#include <perception_msgs/Obstacle.h>

namespace perception_msgs{
    const int UNDEFINED = 99999;
    const uint8_t UNDEFINED_INT8 = -1;
    void initObstacle(perception_msgs::Obstacle& obs);

    bool has_id(perception_msgs::Obstacle& obs){
        return obs.id != UNDEFINED;
	}
    bool has_kind_name(perception_msgs::Obstacle& obs){
        return obs.kind_name != "UNDEFINED";
    }
    bool has_kind(perception_msgs::Obstacle& obs){
        return obs.kind != UNDEFINED_INT8;
    }
    bool has_score(perception_msgs::Obstacle& obs){
	    return obs.score != UNDEFINED;
	}
    bool has_truncated(perception_msgs::Obstacle& obs){
		return obs.truncated != UNDEFINED_INT8;
	}
    bool has_occluded(perception_msgs::Obstacle& obs){
		return obs.occluded != UNDEFINED_INT8;
	}
    bool has_disp_min(perception_msgs::Obstacle& obs){
		return obs.disp_min != UNDEFINED_INT8;
	}
    bool has_disp_max(perception_msgs::Obstacle& obs){
		return obs.disp_max != UNDEFINED_INT8;
	}
    bool has_elevated(perception_msgs::Obstacle& obs){
		return obs.elevated != UNDEFINED_INT8;
	}
    bool has_height(perception_msgs::Obstacle& obs){
		return obs.height != UNDEFINED;
	}
    bool has_width(perception_msgs::Obstacle& obs){
		return obs.width != UNDEFINED;
	}
    bool has_length(perception_msgs::Obstacle& obs){
		return obs.length != UNDEFINED;
	}
    bool has_bbox(perception_msgs::Obstacle& obs){
		return obs.bbox.x_offset != UNDEFINED;
	}
    bool has_location(perception_msgs::Obstacle& obs){
		return obs.location.x != UNDEFINED;
	}
    bool has_alpha(perception_msgs::Obstacle& obs){
		return obs.alpha != UNDEFINED;
	}
    bool has_yaw(perception_msgs::Obstacle& obs){
		return obs.yaw != UNDEFINED;
	}
    bool has_velocity(perception_msgs::Obstacle& obs){
		return obs.velocity.linear.x != UNDEFINED;
    }
}
#endif
