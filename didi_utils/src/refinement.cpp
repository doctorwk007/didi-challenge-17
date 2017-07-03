#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <perception_msgs/ObstacleList.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/tracking/kalman_filters.hpp"

using namespace std;
using namespace cv;
using namespace cv::tracking;

//parameters to refine the detection
double max_length, min_length, max_width, min_width, max_height, min_height;

const double car_width = 1.85;
const double yusuf_height = 1.72;
const double velo_height = 1.72;

ros::Publisher interpolated_pub;
image_transport::Publisher pub;

double cell_size, grid_dim;

//CHAPU-YAW
int yaw_interval = 5;
int yaw_counter = 0;
std::vector <double> last_yaws;

//KALMAN
class TrackedCar: public UkfSystemModel
{
public:
    void stateConversionFunction(const Mat& x_k, const Mat& u_k, const Mat& v_k, Mat& x_kplus1)
    {
        //v_k noise
        x_kplus1.at<double>(0,0) = x_k.at<double>(0,0) + x_k.at<double>(1,0) + v_k.at<double>(0,0);
        x_kplus1.at<double>(1,0) = x_k.at<double>(1,0) + v_k.at<double>(1,0);
    }
    void measurementFunction(const Mat& x_k, const Mat& n_k, Mat& z_k)
    {
        z_k.at<double>(0,0) = x_k.at<double>(0,0) + n_k.at<double>(0,0);
    }
};

std::vector< Ptr<TrackedCar> > models;

std::map < int, cv::Ptr<UnscentedKalmanFilter> > uncsentedKalmanFilterVector;
// cv::Ptr<UnscentedKalmanFilter> uncsentedKalmanFilter;
int DP = 2; //state
int MP = 1; //measurementFunction
int CP = 0; //control
int type = CV_64F;

double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

map < int, list<double> > yaws_list;


double movingMean(int id, double estimation){

    std::map< int, list<double> >::iterator it;
    it = yaws_list.find(id);
    list <double>* last_yaws;
    if (it == yaws_list.end()){
        // ROS_INFO("Creating new agent mean");
        yaws_list[id].push_back(estimation);
        last_yaws = &yaws_list[id];
    }else{
        last_yaws = &yaws_list[id];
        last_yaws->push_back(estimation);
    }
    while (last_yaws->size()>5)
    {
        last_yaws->pop_front();
    }

    double acc = 0.;
    int nele = 0;
    for(auto yaw : *last_yaws){
        acc += yaw;
        nele++;
    }

    return acc/nele;

}

double prefilterYaw(double state, double estimation){
    static const double half_tol = M_PI/4.0;
    // static const double full_tol = M_PI/4.0;

    double temp_est;
    if (fabs(estimation-2*M_PI-state) < fabs(estimation-state)){
        temp_est = estimation-2*M_PI;
        // ROS_ERROR("Filtering 360 error from %lf to %lf (prev %lf)", estimation, temp_est, state);
    }else if (fabs(estimation+2*M_PI-state) < fabs(estimation-state)){
        temp_est = estimation+2*M_PI;
        // ROS_ERROR("Filtering 360 error from %lf to %lf (prev %lf)", estimation, temp_est, state);
    }else{
        temp_est = estimation;
        // ROS_WARN("Keeping %lf (prev %lf)", estimation, state);
    }



    if ((fabs(state-temp_est)>M_PI-half_tol-0.001) && (fabs(state-temp_est)<M_PI+half_tol+0.001)){
        // ROS_ERROR("Filtering 180 error from %lf to %lf (prev %lf)", temp_est, constrainAngle(temp_est-M_PI), state);
        return constrainAngle(temp_est-M_PI);
    }else{
        return temp_est;
    }



    // if (fabs(state-estimation)>2*M_PI-full_tol-0.001){
    //     ROS_ERROR("Filtering 360 error from %lf to %lf (prev %lf)", estimation, estimation-(estimation>0)*2*M_PI+(estimation<0)*2*M_PI, state);
    //     estimation-(estimation>0)*2*M_PI+(estimation<0)*2*M_PI;
    //     // ROS_INFO("%lf", estimation-(estimation>0)*2*M_PI+(estimation<0)*2*M_PI);
    //     return estimation-(estimation>0)*2*M_PI+(estimation<0)*2*M_PI;
    // }else if ((fabs(state-estimation)>M_PI-half_tol-0.001) && (fabs(state-estimation)<M_PI+half_tol+0.001)){
    //     ROS_ERROR("Filtering 180 error from %lf to %lf (prev %lf)", estimation, constrainAngle(estimation-M_PI), state);
    //     // ROS_INFO("%lf %lf", estimation-M_PI, constrainAngle(estimation-M_PI));
    //     return constrainAngle(estimation-M_PI);
    // }else{
    //     ROS_WARN("Keeping %lf (prev %lf)", estimation, state);
    //     return estimation;
    // }
}

int initNewUKF(int id, double init_angle){
    Ptr<TrackedCar> model(new TrackedCar());

    //KALMAN
    const double alpha = 1e-3;//1.5;
    const double beta = 2.0;
    const double kappa = 0.0;

    UnscentedKalmanFilterParams params( DP, MP, CP, 0, 0, model );

    Mat processNoiseCov = Mat::zeros( DP, DP, type );
    double acc_var = pow(M_PI/8.0, 2.0);
    processNoiseCov.at<double>(0, 0) = 1/4.0 * acc_var;
    processNoiseCov.at<double>(0, 1) = 1/2.0 * acc_var;
    processNoiseCov.at<double>(1, 0) = 1/2.0 * acc_var;
    processNoiseCov.at<double>(1, 1) = acc_var;

    Mat measurementNoiseCov = Mat::zeros( MP, MP, type );
    measurementNoiseCov.at<double>(0, 0) = pow(M_PI/4.0, 2.0);

    Mat P = acc_var * Mat::eye( DP, DP, type );

    Mat initState( DP, 1, type );
    initState.at<double>(0, 0) = init_angle;

    params.errorCovInit = P;
    params.measurementNoiseCov = measurementNoiseCov;
    params.processNoiseCov = processNoiseCov;
    params.stateInit = initState.clone();

    params.alpha = alpha;
    params.beta = beta;
    params.k = kappa;

    uncsentedKalmanFilterVector[id] = createUnscentedKalmanFilter( params );
}

// OTRO KALMAN
std::map <int, KalmanFilter> KFlist;
Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
Mat processNoise(2, 1, CV_32F);
Mat measurement = Mat::zeros(1, 1, CV_32F);

int initKF(int id, double init_angle){
    KFlist[id] = KalmanFilter(2, 1, 0);
    KFlist[id].transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

    setIdentity(KFlist[id].measurementMatrix);
    setIdentity(KFlist[id].processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFlist[id].measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KFlist[id].errorCovPost, Scalar::all(1));

    // KFlist[id].statePre.at<float>(0) = init_angle;
    // KFlist[id].statePre.at<float>(1) = 0;

    KFlist[id].statePost.at<float>(0) = init_angle;
    KFlist[id].statePost.at<float>(1) = 0;

}


void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationRadians, cv::Scalar color)
{
    //cv::Scalar color = cv::Scalar(255.0, 255.0, 255.0); // white

    double rotationDegrees = rotationRadians / M_PI * 180.0;
    rotationDegrees = -90-rotationDegrees;

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

    for (int i = 0; i < 4; i++)
        cv::line(image, vertices[i], vertices[(i+1)%4], color);

}

void detection_sub_callback(const perception_msgs::ObstacleListConstPtr msg){

    // TODO: Accumulated yaw computation

    perception_msgs::ObstacleList refined_obs;
    refined_obs.header = msg->header;

    cv::Mat img(700,700,CV_8UC3, CV_RGB(0,0,0));

    double meanYaw;

    for(int obs_idx = 0; obs_idx < msg->obstacles.size(); obs_idx++){

        perception_msgs::Obstacle temp;
        temp = msg->obstacles[obs_idx];
        temp.yaw = msg->obstacles[obs_idx].alpha;
        // ROS_INFO("%lf", temp.yaw);

        double yaw = msg->obstacles[obs_idx].yaw;
        static double last_yaw = -1000;

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
            // cout<< "ORIGINAL WIDTH: " << width << endl;
            // cout<< "ORIGINAL LENGTH: " << length << endl;
            // cout<< "ESTIMATED WIDTH: " << car_width << endl;
            // cout<< "ESTIMATED LENGTH: " << estimated_length << endl;

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

            // cout<< "FINAL WIDTH: " << temp.height << endl;
            // cout<< "FINAL LENGTH: " << temp.length << endl;

            double x_inc = length_diff*cos(temp.yaw);
            double y_inc = length_diff*sin(temp.yaw);
            // cout << "Yaw " << temp.yaw << " Length diff " << length_diff << " cos " << cos(temp.yaw) << endl;
            // cout << "initial pose x and y:" << temp.location.x << " " << temp.location.y << " increments in x and y: " << x_inc << " " << y_inc << endl;

            temp.location.x+=x_inc;
            temp.location.y+=y_inc;
            temp.location.z=temp.height/2.-velo_height;

            double map_size = grid_dim/cell_size;

            // ROS_INFO("%lf %lf %lf %lf %lf", map_size, -(temp.location.y/cell_size)+map_size/2, -(temp.location.x/cell_size)+map_size/2, temp.width/cell_size, temp.height/cell_size);

            temp.bbox.width = temp.length/cell_size;
            temp.bbox.height = temp.width/cell_size;

            temp.bbox.x_offset = -(temp.location.y/cell_size)+map_size/2 - temp.bbox.width/2.0;
            temp.bbox.y_offset = -(temp.location.x/cell_size)+map_size/2 - temp.bbox.height/2.0;

            // KALMAN
            // std::map< int, cv::Ptr<UnscentedKalmanFilter> >::iterator it;
            // it = uncsentedKalmanFilterVector.find(temp.id);
            // if (it == uncsentedKalmanFilterVector.end())
            //     ROS_INFO("Creating new UKF");
            //     initNewUKF(temp.id, temp.yaw);
            //
            // cv::Ptr<UnscentedKalmanFilter> uncsentedKalmanFilter = uncsentedKalmanFilterVector[temp.id];
            //
            // Mat correctStateAUKF( DP, 1, type );
            // Mat measurement( MP, 1, type );
            // Mat prevState( DP, 1, type );
            //
            // prevState = uncsentedKalmanFilter->getState();
            // double prevYaw = prevState.at<double>(0,0);
            // ROS_WARN("[%d] prevYaw: %lf", temp.id, prevYaw);
            //
            // measurement.at<double>(0,0) = prefilterYaw(prevYaw, temp.yaw);
            //
            // uncsentedKalmanFilter->predict();
            // correctStateAUKF = uncsentedKalmanFilter->correct(measurement);

            double new_yaw;
            if (last_yaw<-400){
                last_yaw = temp.yaw;
                new_yaw = last_yaw;
            }else{
                new_yaw =  prefilterYaw(last_yaw, temp.yaw);
                last_yaw = new_yaw;
            }

            meanYaw = movingMean(temp.id, new_yaw);


            // ROS_WARN("[%d] YAW: %lf, FILTERED: %lf, MEAN: %lf", temp.id, temp.yaw, correctStateAUKF.at<double>(0,0), meanYaw);


            // OTRO KALMAN
            // std::map< int, KalmanFilter >::iterator it;
            // it = KFlist.find(temp.id);
            // if (it == KFlist.end())
            //     ROS_INFO("Creating new KF");
            //     initKF(temp.id, temp.yaw);
            //
            // KalmanFilter* KF = &KFlist[temp.id];
            //
            // Mat prediction = KF->predict();
            // float prevYaw = prediction.at<float>(0);
            // ROS_INFO("STATE: %lf", prevYaw);
            //
            // Mat measurement = Mat::zeros(1, 1, CV_32F);
            //
            // measurement.at<float>(0,0) = prefilterYaw(prevYaw, temp.yaw);
            // Mat estimated = KF->correct(measurement);


            // ROS_INFO("[%d] YAW: %f, FILTERED: %f, MEAN: %lf", temp.id, temp.yaw, estimated.at<float>(0,0), meanYaw);

            // Chapuza yaw
            // static const Eigen::Vector3d front_direction(1.0,0.0,0.0);
            //

            //
            // double accumulated_yaw = 0.;
            // for(auto pose : poses_to_yaw){
            //     Eigen::Vector3d car_vector(temp.location.x-pose.obstacles[j].location.x,temp.location.y-pose.obstacles[j].location.y,0); // TODO CHECK
            //     double yaw=angle_between(car_vector,front_direction);
            //     last_yaws.push_back(yaw);
            //     accumulated_yaw+=yaw;
            // }
            // std::sort(last_yaws.begin(), last_yaws.end());
            // double chapu_yaw= (poses_to_yaw.size() == 0 ? 0 : accumulated_yaw/poses_to_yaw.size());

            //OTRO KALMAN temp.yaw = estimated.at<float>(0,0);
            //correctStateAUKF.at<double>(0,0);
        }
        else if(temp.kind_name=="Pedestrian"){
            temp.height=yusuf_height;
            temp.location.z=temp.height/2.-velo_height;
        }
        refined_obs.obstacles.push_back(temp);
        // DrawRotatedRectangle(img, cv::Point(temp.bbox.x_offset + temp.bbox.width/2.0, temp.bbox.y_offset + temp.bbox.height/2.0), cv::Size(temp.bbox.width, temp.bbox.height), temp.yaw, cv::Scalar(0,0,255));
        temp.yaw = meanYaw;
        // DrawRotatedRectangle(img, cv::Point(temp.bbox.x_offset + temp.bbox.width/2.0, temp.bbox.y_offset + temp.bbox.height/2.0), cv::Size(temp.bbox.width, temp.bbox.height), temp.yaw, cv::Scalar(0,255,0));
    }
    interpolated_pub.publish(refined_obs);

    pub.publish(cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());

    // cv::imshow("rotated", img);
    // cv::waitKey(3);

    // CHAPU-YAW
    // poses_to_yaw.push_back(*msg);
    // if(yaw_counter>=yaw_interval){
    //     poses_to_yaw.pop_front();
    // }
    // yaw_counter++;
}

int main(int argc, char* argv[]){

    ros::init(argc,argv,"refinement");
    ros::NodeHandle public_nh, private_nh("~");
    image_transport::ImageTransport it(private_nh);

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

    detection_sub = public_nh.subscribe(detection_sub_topic, 1000,detection_sub_callback);
    interpolated_pub = public_nh.advertise<perception_msgs::ObstacleList>(interpolated_pub_topic,1);
    pub = it.advertise("/rotated_bboxes", 1);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}
