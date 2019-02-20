#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //Process the image here.


    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");


    cv::Mat bgr_image = cv_ptr->image;
    cv::Mat mask;
    // specify the range of colours that you want to include, you can play with the borders here
    cv::Scalar lowerb = cv::Scalar(0,0,254);
    cv::Scalar upperb = cv::Scalar(0,0,255); 

    cv::inRange(bgr_image, lowerb, upperb, mask); // if the frame has any orange pixel, this will be painted in the mask as white

    imshow("mask", mask); // show where orange pixels are located, then use this mask for further processing pass it for example to findNonZero() function in order to obtain the location of the pixels, etc...



    // cv::imshow("view", cv_ptr->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/bounding_box_camera/RGB", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}