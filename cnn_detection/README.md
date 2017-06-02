### ** Package summary ** ###
The *cnn_detection* package implements an object detection algorithm based on Convolutional Neural Networks (deep learning), using the [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn) framework.

### ** Inputs (Subscribed Topics) ** ###
* stereo_camera/left/image_rect_color (sensor_msgs/Image)

*Extra topics could be required depending on the selected settings.*

### ** Outputs (Published Topics) ** ###
* cnn_detection/filtered_detection (cnn_detection/ObstacleList)

*More topics are generated along the process.*

### ROS dependencies ###
* [std_msgs](http://wiki.ros.org/std_msgs)
* [sensor_msgs](http://wiki.ros.org/sensor_msgs)
* [image_transport](http://wiki.ros.org/image_transport)
* [cv_bridge](http://wiki.ros.org/cv_bridge)
* [message_generation](http://wiki.ros.org/message_generation)
* [pcl_ros](http://wiki.ros.org/pcl_ros)

### System dependencies ###
* [PCL](http://pointclouds.org/).
