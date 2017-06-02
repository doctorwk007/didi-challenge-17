#ifndef CLOUD_FILTER_HPP
#define CLOUD_FILTER_HPP

#include <memory>
#include <cmath>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/console/time.h>

#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>

class CloudFilter
{
public:
    CloudFilter();
    CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);
    ~CloudFilter();

    void setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    void setCarName(std::string &car_name);

    /* Remove the points that are not in the camera FOV (angle in degrees)*/
    void filterFOV(double horizontal_fov);

    /* Remove the floor points using a heightmap algorithm */
    void removeFloor(double cell_size, double height_threshold, double max_height, int grid_dim);

    /* get the 2D grid birdview of the cloud with 3 channels (height, density, intensity) */
    std::shared_ptr<cv::Mat> birdView(double cell_size, double max_height, double grid_dim);

    /* Remove all points below the given intensity threshold */
    void filterIntensities(double intensity_threshold);

    /* Separate the pointcloud into clusters */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterViewer(double intensity_threshold);

    /* Wait for the transform velodyne -> camera and update velo_cam_transform_ */
    void initTF(std::string velodyne_frame, std::string camera_frame);
    
    //load values to the max points map, curretnly computing everything, is it better to load from a file?
    void init_max_points_map(int planes, float cell_size, float h_res, int grid_dim, float low_opening, float v_res);

private:
    // Cloud to be processed and updated in place
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_;

    // Transform between the veloyne and the camera. Needed to remove points out of the camera range.
    tf::StampedTransform velo_cam_transform_;
    // Transform between the veloyne and the camera. Needed to know the height
    tf::StampedTransform base_velo_transform_;
    // Trasform listener to get the TFs
    tf::TransformListener* tf_;

    std::string car_;
    
    //matrix to keep the map of the max points that each cell can have
    float max_points_map_[1000][1000];  //have to change this to make it the size of the grid
    //std::vector<std::vector<float>> max_points_map_;

    /* Return true if the point is inside the camera FOV. Used to filter points in filterFOV */
    bool pointInCameraFov(pcl::PointXYZI p, double horizontal_fov);

    /* Return true if the point is inside a cell considered not ground. Used to filter points in removeFloor */
    bool filterGround(pcl::PointXYZI p,int grid_dim,const std::vector<std::vector<float> > &min,const std::vector<std::vector<float> > &max,const std::vector<std::vector<float> > &init ,const double &height_threshold,const double &cell_size);

};

#endif
