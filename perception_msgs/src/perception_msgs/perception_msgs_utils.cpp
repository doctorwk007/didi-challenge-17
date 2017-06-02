#include <perception_msgs/perception_msgs_utils.hpp>

namespace perception_msgs {
    void initObstacle(perception_msgs::Obstacle& obs){
        obs.id = UNDEFINED;
        obs.kind = UNDEFINED_INT8;
        obs.kind_name = "UNDEFINED";
        obs.score = UNDEFINED;
        obs.truncated = UNDEFINED_INT8;
        obs.occluded = UNDEFINED_INT8;
        obs.disp_min = UNDEFINED_INT8;
        obs.disp_max = UNDEFINED_INT8;
        obs.elevated = UNDEFINED_INT8;
        obs.height = UNDEFINED;
        obs.width = UNDEFINED;
        obs.length = UNDEFINED;
        obs.bbox.x_offset = UNDEFINED;
        obs.bbox.y_offset = UNDEFINED;
        obs.bbox.height = UNDEFINED;
        obs.bbox.width = UNDEFINED;
        obs.bbox.do_rectify = false;
        obs.location.x = UNDEFINED;
        obs.location.y = UNDEFINED;
        obs.location.z = UNDEFINED;
        obs.alpha = UNDEFINED;
        obs.yaw = UNDEFINED;
        obs.velocity.linear.x = UNDEFINED;
        obs.velocity.linear.y = UNDEFINED;
        obs.velocity.linear.z = UNDEFINED;
        obs.velocity.angular.x = UNDEFINED;
        obs.velocity.angular.y = UNDEFINED;
        obs.velocity.angular.z = UNDEFINED;
    }
}
