#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <perception_msgs/ObstacleList.h>

using namespace std;

//parameters to refine the detection
double max_length, min_length, max_width, min_width, max_height, min_height;

const double car_width = 1.85;
const double yusuf_height = 1.72;
const double velo_height = 1.72;

ros::Publisher interpolated_pub;

double cell_size, grid_dim;

void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees)
{
    cv::Scalar color = cv::Scalar(255.0, 255.0, 255.0) // white

    // Create the rotated rectangle
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image,
                       vertices,
                       4,
                       color);
}

void detection_sub_callback(const perception_msgs::ObstacleListConstPtr msg){

    // TODO: Accumulated yaw computation

    perception_msgs::ObstacleList refined_obs;
    refined_obs.header = msg->header;

    for(int obs_idx = 0; obs_idx < msg->obstacles.size(); obs_idx++){

        perception_msgs::Obstacle temp;
        temp = msg->obstacles[obs_idx];
        temp.yaw = msg->obstacles[obs_idx].alpha;
        ROS_INFO("%lf", temp.yaw);

        double yaw = msg->obstacles[obs_idx].yaw;

        if(msg->obstacles[obs_idx].kind_name=="Car"){

            double height, width, length;
            double height_diff = 0, length_diff = 0;
            double estimated_length;

            height = msg->obstacles[obs_idx].height;

            //length and width may be interchanged depending on the yaw angle
            if((0 < abs(yaw) < M_PI/4) || (3*M_PI/4 < abs(yaw) < M_PI)){
                length = msg->obstacles[obs_idx].length;
                width = msg->obstacles[obs_idx].width;
            }else{
                width = msg->obstacles[obs_idx].length;
                length = msg->obstacles[obs_idx].width;
            }

            estimated_length = length-fabs(width*sin(temp.yaw));
            cout<< "ORIGINAL WIDTH: " << width << endl;
            cout<< "ORIGINAL LENGTH: " << length << endl;
            cout<< "ESTIMATED WIDTH: " << car_width << endl;
            cout<< "ESTIMATED LENGTH: " << estimated_length << endl;

            //oriented length and width stimation
            width = car_width;

            temp.width = width;
            length = estimated_length;

            //puth thresholds in the dimensions and get increment
            if(height<min_height) {
                height_diff=(min_height-height)/2;
                height=min_height;
            }
            else if(height>max_height){
                height_diff=(max_height-height)/2;
                height=max_height;
            }
            if(length<min_length){
                length_diff=(min_length-length)/2;
                length=min_length;
            }
            else if(length>max_length){
                length_diff=(max_length-length)/2;
                length=max_length;
            }
            // Update size values
            temp.length=length;
            temp.height=height;

            cout<< "FINAL WIDTH: " << temp.height << endl;
            cout<< "FINAL LENGTH: " << temp.length << endl;

            double x_inc = length_diff*cos(temp.yaw);
            double y_inc = length_diff*sin(temp.yaw);
            cout << "Yaw " << temp.yaw << " Length diff " << length_diff << " cos " << cos(temp.yaw) << endl;
            cout << "initial pose x and y:" << temp.location.x << " " << temp.location.y << " increments in x and y: " << x_inc << " " << y_inc << endl;

            temp.location.x+=x_inc;
            temp.location.y+=y_inc;
            temp.location.z=temp.height/2.-velo_height;

            double map_size = grid_dim/cell_size;

            ROS_INFO("%lf %lf %lf %lf %lf", map_size, -(temp.location.y/cell_size)+map_size/2, -(temp.location.x/cell_size)+map_size/2, temp.width/cell_size, temp.height/cell_size);

            temp.bbox.width = temp.length/cell_size;
            temp.bbox.height = temp.width/cell_size;

            temp.bbox.x_offset = -(temp.location.y/cell_size)+map_size/2 - temp.bbox.width/2.0;
            temp.bbox.y_offset = -(temp.location.x/cell_size)+map_size/2 - temp.bbox.height/2.0;

        }
        else if(temp.kind_name=="Pedestrian"){
            temp.height=yusuf_height;
            temp.location.z=temp.height/2.-velo_height;
        }
        refined_obs.obstacles.push_back(temp);
    }
    interpolated_pub.publish(refined_obs);
}

int main(int argc, char* argv[]){

    ros::init(argc,argv,"refinement");
    ros::NodeHandle public_nh, private_nh("~");

    std::string detection_sub_topic, interpolated_pub_topic;

    private_nh.param<string>("detection_sub_topic", detection_sub_topic, "smart_tracking/obstacles");
    private_nh.param<string>("interpolated_pub_topic", interpolated_pub_topic, "refined_detection");
    private_nh.param("/birdview/cell_size", cell_size, 0.1);
    private_nh.param("/birdview/grid_dim", grid_dim, 0.70);


    private_nh.param("max_height",max_height,1.65);
    private_nh.param("min_height",min_height,1.40);
    private_nh.param("max_width",max_width,1.8);
    private_nh.param("min_width",min_width,1.67);
    private_nh.param("max_length",max_length,4.52);
    private_nh.param("min_length",min_length,4.26);

    ros::Subscriber detection_sub;

    detection_sub = public_nh.subscribe(detection_sub_topic, 0,detection_sub_callback);
    interpolated_pub = public_nh.advertise<perception_msgs::ObstacleList>(interpolated_pub_topic,1);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}
