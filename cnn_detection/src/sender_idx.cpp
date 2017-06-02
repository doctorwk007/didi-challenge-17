#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

class ImageSender
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle *priv_nh_;
    image_transport::ImageTransport it_;

    message_filters::Subscriber<sensor_msgs::Image> im_left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> head_sub_;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExSync;
    message_filters::Synchronizer<ExSync> sync;

    int sockfd, newsockfd, portno;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    bool wait_disparity, disproj_;

    ImageSender(ros::NodeHandle * nh_pt, const char * right_topic):
        priv_nh_(nh_pt),
        it_(nh_),
        head_sub_(nh_, right_topic, 1),
        im_left_sub_(nh_, "/stereo_camera/left/image_rect_color", 1),
        sync(ExSync(1), im_left_sub_, head_sub_)
    {

        nh_.param<int>("port", portno, 3050);
        priv_nh_->param<bool>("wait_disparity", wait_disparity, false);
        priv_nh_->param<bool>("four_channels", disproj_, false);

        if (wait_disparity){
          sync.registerCallback(boost::bind(&ImageSender::callback, this, _1, _2));
        }else{
          sync.registerCallback(boost::bind(&ImageSender::send, this, _1));
        }

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) ROS_FATAL("ERROR opening socket");
        int enable = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
            ROS_FATAL("setsockopt(SO_REUSEADDR) failed");
        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(portno);
        if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
            ROS_FATAL("ERROR on binding");
            ros::shutdown();
            exit(-1);
        }
        listen(sockfd,5);
        clilen = sizeof(cli_addr);
        try{
            newsockfd = accept(sockfd,
                         (struct sockaddr *) &cli_addr,
                         &clilen);
        }catch ( int ex ){
            ROS_FATAL ("Error accepting socket");
            exit(-1);
        }

        if (newsockfd < 0) ROS_FATAL("ERROR on accept");

        ROS_INFO("[Sender] Ready");
    }

    void callback(const sensor_msgs::ImageConstPtr& left_im, const sensor_msgs::ImageConstPtr& sel_head)
    {
        if (sel_head->header.stamp != left_im->header.stamp) return;
        send(left_im);
    }

    void send(const sensor_msgs::ImageConstPtr& left_im){

        unsigned char buf[10];
        static unsigned char time[20];
        static unsigned char toSend[3686400];
        static unsigned char e_size[8];
        static unsigned char e_width[5], e_height[5];
        int chn = disproj_ ? 4 : 3;
        int32_t size = left_im->height*left_im->width*chn;
        int n;
        char *sendable_size = (char*)&size;

        sprintf((char*)e_size, "%08d", size);
        n = write(newsockfd, e_size, 8);
        if (n < 0) ROS_ERROR("ERROR writing to socket");

        // CvBridge is used just in case images are BGRA
        cv::Mat img;

        try
        {
            if (disproj_){
              img = cv_bridge::toCvShare(left_im, "bgra8")->image;
            }else{
              img = cv_bridge::toCvShare(left_im, "bgr8")->image;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("CvBridge failed");
        }

        sprintf((char*)time, "%09d%10d", left_im->header.stamp.sec, left_im->header.stamp.nsec);
        sprintf((char*)e_width, "%04d", left_im->width);
        sprintf((char*)e_height, "%04d", left_im->height);

        memcpy(toSend, time, 20);
        memcpy(toSend+20, e_width, 4);
        memcpy(toSend+24, e_height, 4);
        memcpy(toSend+28, &img.data[30], size-30);

        n = read(newsockfd,buf,3);
        if (n < 0) ROS_ERROR("ERROR reading from socket");

        ROS_INFO("[Sender] Sending with stamp %.3lf at %.3lf", left_im->header.stamp.toSec(), ros::Time::now().toSec());
        n = write(newsockfd, toSend, size);
        if (n < 0) ROS_ERROR("ERROR writing to socket");

        n = read(newsockfd,buf,3);
        if (n < 0) ROS_ERROR("ERROR reading from socket");

    }

    void shutdown(){
        close(newsockfd);
        close(sockfd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_sender_demo");
    ros::NodeHandle priv_nh_("~");

    bool sync_selected;

    priv_nh_.param<bool>("sync_selected", sync_selected, false);
    if (sync_selected){
      ROS_INFO("CNN Stream will wait for disparity-processed frames");
    }

    ImageSender is(&priv_nh_, sync_selected?"/stereo_camera/selected_image":"/stereo_camera/left/image_rect_color");

    while (ros::ok()){
      ros::spin();
    }

    is.shutdown();
}
