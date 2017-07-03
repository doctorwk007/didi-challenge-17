#ifndef RADAR_TRACK_HPP
#define RADAR_TRACK_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using namespace std;
struct COORDS{
    double x;
    double y;
};

class RadarTrack
{
public:
    COORDS current_;
    COORDS prev_;
    long id_;

    RadarTrack(cv::Point2d track){
        id_ = -1;
        current_.x = track.x;
        current_.y = track.y;
        prev_.x = -99999;
        prev_.y = -99999;
    }

    bool has_prev(){
        return prev_.x!= -99999;
    }
};
#endif
