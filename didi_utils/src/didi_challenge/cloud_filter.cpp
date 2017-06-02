#include "didi_challenge/cloud_filter.hpp"


CloudFilter::CloudFilter(){}

CloudFilter::CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    setInputCloud(input_cloud);
}

CloudFilter::~CloudFilter()
{
    cloud_ptr_.reset();
}

void CloudFilter::setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    cloud_ptr_.reset();
    cloud_ptr_ = input_cloud;
}

/* Return true if the point is inside the camera FOV. Used to filter points in filterFOV */
bool CloudFilter::pointInCameraFov(pcl::PointXYZI p, double horizontal_fov)
{
    // Translate the point to the camera frame
    p.x -= velo_cam_transform_.getOrigin().x();
    p.y -= velo_cam_transform_.getOrigin().y();

    // Automatically remove points behind the car or too far
    //if (p.x < 0 || p.x > 100) return false;   //already done
    if (p.x < 0 || p.x > 100) return false;

    //double angle = (p.x != 0.0) ? std::atan(p.y / p.x) : 0.0;
    //return (abs(angle) < (horizontal_fov / 2.0));

    // Instead of computing the atan, just compare the relation between x/y
    return (abs(p.y) < (horizontal_fov/90.0) * p.x);
}

/* Remove the points that are not in the camera FOV */
void CloudFilter::filterFOV(double horizontal_fov)
{
    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&](const pcl::PointXYZI& p){return !pointInCameraFov(p, horizontal_fov);}),
                             cloud_ptr_->points.end()
                             );
    cloud_ptr_->width = cloud_ptr_->points.size();
}

/* Remove all points above the given intensity threshold */
bool CloudFilter::filterGround(pcl::PointXYZI p, int grid_dim, const std::vector<std::vector<float> > &min, const std::vector<std::vector<float> > &max, const std::vector<std::vector<float> > &init , const double &height_threshold, const double &cell_size)
{
    int x = ((grid_dim/2)+p.x/cell_size);
    int y = ((grid_dim/2)+p.y/cell_size);
    if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && init[x][y])
        return ((max[x][y] - min[x][y] < height_threshold) );
}

/* Remove the floor points using a heightmap algorithm */
void CloudFilter::removeFloor(double cell_size, double height_threshold, double max_height,int grid_dim)
{

    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    std::vector<std::vector<float> > min;min.resize(grid_dim,std::vector<float>(grid_dim, 0));
    std::vector<std::vector<float> > max;max.resize(grid_dim,std::vector<float>(grid_dim, 0));
    std::vector<std::vector<float> > init;init.resize(grid_dim,std::vector<float>(grid_dim, 0));


    // build height map
    for (unsigned i = 0; i < cloud_ptr_->points.size(); ++i) {
        int x = ((grid_dim/2)+cloud_ptr_->points[i].x/cell_size);
        int y = ((grid_dim/2)+cloud_ptr_->points[i].y/cell_size);
        if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim) {
            if (!init[x][y]) {
                min[x][y] = cloud_ptr_->points[i].z;
                max[x][y] = cloud_ptr_->points[i].z;
                init[x][y] = true;
            } else {
                min[x][y] = std::min(min[x][y], cloud_ptr_->points[i].z);
                max[x][y] = std::max(max[x][y], cloud_ptr_->points[i].z);
            }
        }
    }

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&](const pcl::PointXYZI& p)
    {return filterGround(p,grid_dim,min,max,init,height_threshold,cell_size);}),
                             cloud_ptr_->points.end()
                             );
    cloud_ptr_->width = cloud_ptr_->points.size();

}

std::shared_ptr<cv::Mat> CloudFilter::birdView(double cell_size, double max_height, double grid_dim)
{
    // TODO: Add an interpolation to fill the unknown cells


    int grid_cells = grid_dim / cell_size;
    std::shared_ptr<cv::Mat> bird_view(new cv::Mat(grid_cells, grid_cells, CV_8UC3));

    int density[grid_cells][grid_cells];
    float height[grid_cells][grid_cells];
    float intensity[grid_cells][grid_cells];

    // Init the grids
    for (int i = 0; i < grid_cells; ++i)
    {
        for (int j = 0; j < grid_cells; ++j)
        {
            height[i][j] = -9999.9;
            density[i][j] = 0;
            intensity[i][j] = 0.0;
        }
    }


    // Fill the grids
    for (const auto& point : cloud_ptr_->points)
    {
        // TODO: Replace 2.5 with the actual velodyne height (from the floor)
        float z = point.z + 2.5;
        if (z < max_height)
        {
            int x = grid_cells/2 - point.x/cell_size;
            int y = grid_cells/2 - point.y/cell_size;

            if (x >= 0 && x < grid_cells && y >= 0 && y < grid_cells)
            {
                // Update cell max height
                height[x][y] = std::max(height[x][y], z);
                // Increment the number of points in the cell
                density[x][y]++;
                // Update the cell intensity sum to later compute the mean
                intensity[x][y] += point.intensity;
            }
        }
    }

    // Fill the bird view mat
    uchar* mat_ptr = bird_view->data;
    for (int i = 0; i < grid_cells; ++i)
    {
        for (int j = 0; j < grid_cells; ++j)
        {
            // Set unknown heights (still in -9999.9) to 0
            height[i][j] = std::max(height[i][j], (float)0.0);
            // Normalize the height to 0-255
            *mat_ptr++ = (uchar)(255 * height[i][j] / max_height);
            // Limit the density to 255 (we should never have that many points in the same cell...)
            int points_cell_pixel=density[i][j]/max_points_map_[i][j]*255;
            *mat_ptr++ = (uchar)std::min(points_cell_pixel,255);

            //            if(density[i][j]/max_points_map_[i][j]*255==0){
            //                //if(density[i][j]!=0||max_points_map_[i][j]!=0)
            //                std::cout<<"X and Y: "<<i<<" "<<j<<" "<<"Points in cell: "<<density[i][j]<<"Possible points: "<<max_points_map_[i][j]<<std::endl;
            //            }

            // Limit the density to 255 (we should never have that many points in the same cell...)
            //*mat_ptr++ = (uchar)std::min(10*density[i][j], 255);

            // Compute the intensity mean for that cell given the cell density and normalize it to 0-255
            float norm_intensity=0;
            if(car_=="kitti")
                norm_intensity = (density[i][j] > 0) ? (255 * intensity[i][j] / density[i][j]) : 0;
            else if(car_=="didi")
                norm_intensity = (density[i][j] > 0) ? ( intensity[i][j] / density[i][j]) : 0;
            else{
                ROS_ERROR("car name is not correct\n");
            }
            *mat_ptr++ = (uchar)norm_intensity;
        }
    }

    return bird_view;
}

/* Remove all points above the given intensity threshold */
void CloudFilter::filterIntensities(double intensity_threshold)
{
    // Resize the cloud to make it non-organized and work faster
    cloud_ptr_->width = cloud_ptr_->width * cloud_ptr_->height;
    cloud_ptr_->height = 1;

    // Avoid pcl filters if possible because they copy the cloud inside.
    // Points can be removed using std algorithms (erase + remove_if)
    cloud_ptr_->points.erase(std::remove_if(cloud_ptr_->points.begin(),
                                            cloud_ptr_->points.end(),
                                            [&intensity_threshold](const pcl::PointXYZI& p){return p.intensity > intensity_threshold;}),
                             cloud_ptr_->points.end()
                             );
    cloud_ptr_->width = cloud_ptr_->points.size();
}

/* Auxiliary function to help decide how good is a clusterizing function*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudFilter::clusterViewer(double intensity_threshold){

    ros::Time init=ros::Time::now();
    //Quick test using euclidian cluster just for visualization and estimation purposes
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_ptr_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.3); // 30cm
    ec.setMinClusterSize (15);
    ec.setMaxClusterSize (2500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_ptr_);
    ec.extract (cluster_indices);
    std::cout<<"time spent in clustering: "<<(ros::Time::now()-init)*1000<< std::endl;

    ros::Duration obb_time(0);
    init=ros::Time::now();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZRGB tempPoint;
        tempPoint.r=rand()%255;
        tempPoint.g=rand()%255;
        tempPoint.b=rand()%255;

        pcl::PointXYZI min_point_OBB;
        pcl::PointXYZI max_point_OBB;
        pcl::PointXYZI position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;


        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back(cloud_ptr_->points[*pit]);
        }

        pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
        feature_extractor.setInputCloud (cloud_cluster);
        feature_extractor.compute ();
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


        //compute oriented bounding box and check if it is possible to be a car, problem, x and y might not be the closest to the velodyne ones, so this should be changed, also still needs a filter for too big clusters
        //2 condition should remove some walls but does not work, tomorrow i will find out why
        if(max_point_OBB.z-min_point_OBB.z>0.5&&max_point_OBB.y-min_point_OBB.y>0.5&&max_point_OBB.z-min_point_OBB.z<5&&max_point_OBB.y-min_point_OBB.y<5)
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                tempPoint.x=cloud_ptr_->points[*pit].x;
                tempPoint.y=cloud_ptr_->points[*pit].y;
                tempPoint.z=cloud_ptr_->points[*pit].z;
                dynamic_cluster_cloud->points.push_back (tempPoint);
            }
    }
    std::cout<<"time spent in OBB: "<<(ros::Time::now()-init)*1000<< std::endl;


    dynamic_cluster_cloud->header=cloud_ptr_->header;

    return dynamic_cluster_cloud;

}

/* Wait for the transform velodyne -> camera and update velo_cam_transform_ */
void CloudFilter::initTF(std::string velodyne_frame, std::string camera_frame)
{
    if (!tf_) tf_ = new tf::TransformListener;
    bool tf_error = true;
    while(tf_error)
    {
        try
        {
            tf_->waitForTransform(velodyne_frame, camera_frame, ros::Time(0), ros::Duration(5));
            tf_->lookupTransform(velodyne_frame, camera_frame, ros::Time(0), velo_cam_transform_);
            tf_->waitForTransform("base_link", velodyne_frame, ros::Time(0), ros::Duration(5));
            tf_->lookupTransform("base_link", velodyne_frame, ros::Time(0), base_velo_transform_);
            tf_error = false;
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
            //throw (ex);
        }
    }
    std::cout << "New transform: " << velo_cam_transform_.getOrigin().x() << ", " << velo_cam_transform_.getOrigin().y() << std::endl;
}

void CloudFilter::init_max_points_map(int planes, float cell_size, float h_res, int grid_dim,float low_opening,float v_res){
    std::cout<<"making max points map: "<<std::endl;
    float cos_h_res=cos(h_res);
    ros::Time init=ros::Time::now();
    int num_plane=0;
    float x,y;
    //std::ofstream myfile;
    //myfile.open ("/home/lsi2/catkin_ws/src/didi_challenge/src/example.txt");
    int grid_cells = grid_dim / cell_size;
    std::cout<<"grid cells: "<<grid_cells<<" Altura velodyne "<<base_velo_transform_.getOrigin().z()<<std::endl;

    for(int i=0;i<grid_cells;i++){
        x=grid_cells*cell_size/2-i*cell_size;
        //myfile<<"\n";
        for(int j=0;j<grid_cells;j++){
            y=grid_cells*cell_size/2-j*cell_size;
            if(j==0&&i==0){
                //ROS_WARN("0 0 point  ");
                max_points_map_[i][j]=1;
            }
            else{
                float distance=pow((pow(x,2)+pow(y,2)),0.5);//get squared distance
                float dispersion=pow(2*pow(distance,2)*(1-cos_h_res),0.5);

                //WARNING, VELODYNE HEIGHT CHANGED BY HAND IN DIDI, TF ARE NOT WELL SET
                float angle;
                if(car_=="kitti")
                    angle=atan(base_velo_transform_.getOrigin().z()/distance)*180/M_PI;
                else if(car_=="didi")
                    angle=atan((base_velo_transform_.getOrigin().z()+0.4)/distance)*180/M_PI;
                else
                    ROS_ERROR("car name not valid\n");


                num_plane= abs((low_opening-angle)/v_res);
                float max_points=(planes-num_plane)*cell_size/dispersion;
                max_points_map_[i][j]=max_points/2;//limit it to a % of the max points possibles,
                if(max_points_map_[i][j]<1){
//                    std::cout<<" i "<<i<<" j "<<j<<"\n";
//                    ROS_WARN("0 points  ");
                    max_points_map_[i][j]=1;
                }

                //std::cout<<"max points: "<<max_points<<"\n";
                //myfile<<(int)max_points_map_[i][j]<<" ";

            }
        }
    }
    std::cout<<x<<"  "<<y<<std::endl;

    //std::cout<<max_points_map_<<std::endl;
    // std::cout<<max_points_map_[980][980]<<std::endl;
    //        myfile.close();
    std::cout<<"Time consumed building the max points map: "<<ros::Time::now()-init<<std::endl;
}

void CloudFilter::setCarName(std::string &car_name){
    car_=car_name;

}
