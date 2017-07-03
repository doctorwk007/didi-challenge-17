#include <didi_challenge/cloud_filter.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <chrono>
#include <sys/stat.h>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>




#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;
using namespace cv;

// Time measurement function to make it easier
/* How to use it:
    call tic() to set the timestamp
    call tic() again to measure
    It returns the time elapsed since the last call to tic()
    This value is also stored in the variable ELAPSED_TIME
*/
std::chrono::high_resolution_clock::time_point _time;
double ELAPSED_TIME = 0.0;
double tic(){
    std::chrono::high_resolution_clock::time_point _t = _time;
    _time = std::chrono::high_resolution_clock::now();
    ELAPSED_TIME = (double)std::chrono::duration_cast<std::chrono::microseconds>(_time - _t).count() / 1000.0;
    return ELAPSED_TIME;
}


image_transport::Publisher bird_view_pub;
ros::Publisher cloud_pub,clusters_pub;
CloudFilter filter;

bool is_online, write_imgs,generate_rotated;

double camera_fov;
double intensity_threshold;

//floor removal parameters
double cell_size,height_threshold,max_height,cell_size_height_map; int grid_dim,grid_dim_height_map;

string rosbag_name;
string path;
string saving_path;
string car;
int rotated_horizontal_fov;


void cloud_callback(const sensor_msgs::PointCloud2Ptr & cloud_msg)
{
    static int num_image=0;
    // Change the intensity field name, so we can use it with pcl point type
    cloud_msg->fields[3].name = "intensity";
    // Convert cloud msg to pcl type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(*cloud_msg, *cloud_ptr);

    // Update the filter cloud
    filter.setInputCloud(cloud_ptr);

    // Remove points out of the camera FOV
    //    tic();
    //    filter.filterFOV(camera_fov);
    //    cout << "FOV filter: " << tic() << endl;

    // Remove floor points
    tic();
    filter.removeFloor(cell_size_height_map,height_threshold,max_height,grid_dim_height_map);
    cout << "Floor filter: " << tic() << endl;


    // tic();
    std::shared_ptr<Mat> bird_view = filter.birdView(cell_size, max_height, grid_dim);
    cout << "Bird view: " << tic() << endl;

    if(is_online){
        // Publish bird_view img
        cv_bridge::CvImage cv_bird_view;
        cv_bird_view.header = cloud_msg->header;
        cv_bird_view.encoding = "bgr8";
        cv_bird_view.image = *bird_view;
        bird_view_pub.publish(cv_bird_view.toImageMsg());
    }

    // Dataset generator
    if(write_imgs){
        //saving original image
        stringstream saving_absolute;
        saving_absolute<<saving_path<<"/";
        saving_absolute<< setfill('0') << setw(6)<<num_image;
        saving_absolute<<".png";
        imwrite( saving_absolute.str(), *bird_view );

        if(generate_rotated){

            //cropping image
            for(int x=0;x<bird_view->cols;x++){
                for(int y=0;y<bird_view->rows;y++){
                    int a=bird_view->rows/2-y;
                    int b=abs(bird_view->cols/2-x);
                    if(a < (rotated_horizontal_fov/90.0)*b||y>bird_view->rows/2){
                        bird_view->at<cv::Vec3b>(y,x)=cv::Vec3b(0,0,0);
                        //cout<<"blacking pixel:"<<x<< " "<<y <<"\n";
                    }
                }
            }
            int rotation_angle=-90;

            //generated the 4 rotations
            for(int num=1;num<5;num++){
                cv::Point2f pc(bird_view->cols/2., bird_view->rows/2.);
                cv::Mat r = cv::getRotationMatrix2D(pc,rotation_angle*(num-1) , 1.0);

                cv::Mat rotated;
                cv::warpAffine(*bird_view, rotated, r, bird_view->size());

                saving_absolute.str("");
                saving_absolute<<saving_path<<"/"<<num;
                saving_absolute<< setfill('0') << setw(5)<<num_image;
                saving_absolute<<".png";
                cv::imwrite(saving_absolute.str(),rotated);
            }

        }
    }

    // Publish the filtered cloud
    cloud_pub.publish(*cloud_ptr);
    //clusters_pub.publish(*dynamic_cluster_cloud);
    num_image++;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "laser_birdview");
    ros::NodeHandle public_nh, private_nh("~");
    image_transport::ImageTransport it(public_nh);

    int planes;double h_res,low_opening,v_res;

    private_nh.param<string>("rosbag_name", rosbag_name, "training_images");
    string cloud_topic;
    private_nh.param<string>("cloud_topic", cloud_topic, "/kitti/velo/pointcloud");
    string output_topic;
    private_nh.param<string>("output_topic", output_topic, "filtered_cloud");
    string velodyne_tf_frame;
    private_nh.param<string>("velodyne_tf_frame", velodyne_tf_frame, "/velo_link");
    string camera_tf_frame;
    private_nh.param<string>("camera_tf_frame", camera_tf_frame, "/camera_color_left");
    private_nh.param("camera_fov", camera_fov, 110.0);
    private_nh.param("intensity_threshold", intensity_threshold, 0.05);
    private_nh.param("planes", planes, 32);
    private_nh.param("h_res", h_res, 0.0034889);
    public_nh.param("cell_size", cell_size, 0.1);
    public_nh.param("/birdview/cell_size", cell_size, 0.1);
    private_nh.param("cell_size_height_map", cell_size_height_map, 0.25);
    private_nh.param("height_threshold", height_threshold, 0.10);
    private_nh.param("max_height", max_height, 3.0);//not used, dont know if useful, there are buses that are quite high
//    public_nh.param("grid_dim", grid_dim, 1000);//300*cell_size = total pointcloud size
    public_nh.param("/birdview/grid_dim", grid_dim, 1000);//300*cell_size = total pointcloud size
    private_nh.param("grid_dim_height_map", grid_dim_height_map, 300);//300*cell_size = total pointcloud size
    private_nh.param("low_opening", low_opening, 24.9);//-24.9 for 64 planes
    private_nh.param("v_res", v_res, 0.4);//0.4 for 64 planes
    private_nh.param<string>("car", car, "kitti");
    private_nh.param("generate_rotated", generate_rotated,false);
    private_nh.param("rotated_horizontal_fov", rotated_horizontal_fov,90);

    // Operating mode
    private_nh.param("is_online", is_online, true);
    private_nh.param("write_imgs", write_imgs, false);

    // Init the velodyne - camera transformation for the filter
    filter.setCarName(car);
    filter.initTF(velodyne_tf_frame, camera_tf_frame);
    filter.init_max_points_map( planes,  cell_size,  h_res, grid_dim,low_opening,v_res);

    // namedWindow("bird view", CV_WINDOW_NORMAL);
    //namedWindow("bird view height", CV_WINDOW_NORMAL);
    //namedWindow("bird view density", CV_WINDOW_NORMAL);
    //namedWindow("bird view intensity", CV_WINDOW_NORMAL);


    if(is_online){
        bird_view_pub = it.advertise("bird_view", 1);
        cloud_pub = public_nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(output_topic, 1);
        clusters_pub = public_nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("clusters_output", 1);
        ros::Subscriber cloud_sub = public_nh.subscribe(cloud_topic, 1000, cloud_callback);

        while (ros::ok())
        {
            waitKey(10);
            ros::spinOnce();
        }

        destroyAllWindows();
    }else{
        string bagpath = rosbag_name;
        saving_path=ros::package::getPath("didi_challenge");
        //rosbag_name=argv[1];
        //cout<<"saving path: "<<saving_path<<" rosbag_name: "<<rosbag_name<<endl;
        rosbag_name.erase(0,rosbag_name.find_last_of("/")+1);
        rosbag_name.erase(rosbag_name.find_last_of("."),string::npos);
        saving_path=saving_path+"/training_images/"+rosbag_name;
        //saving_path="/media/datasets/didi-challenge/training/"+rosbag_name;
        cout << "Saving imgs in: " << saving_path << endl;
        mkdir(saving_path.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        rosbag::Bag bag;
        cout << "Opening bag file: " << bagpath << endl;
        //cout << "bag name " << rosbag_name << endl;
        bag.open(bagpath, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("/kitti/velo/pointcloud"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));
        int frames = 0;
        foreach(rosbag::MessageInstance const m, view){
            sensor_msgs::PointCloud2Ptr msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg != NULL){
                frames++;
                cloud_callback(msg);
            }
        }

        bag.close();
        cout << "Total frames " << frames << endl;
    }
    return 0;
}
