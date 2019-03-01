#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <flightgoggles/IRMarker.h>
#include <flightgoggles/IRMarkerArray.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//   try
//   {
//     //Process the image here.


//     cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");


//     cv::Mat bgr_image = cv_ptr->image;
//     cv::Mat mask;
//     // specify the range of colours that you want to include, you can play with the borders here
//     cv::Scalar lowerb = cv::Scalar(0,0,254);
//     cv::Scalar upperb = cv::Scalar(0,0,255); 

//     cv::inRange(bgr_image, lowerb, upperb, mask); // if the frame has any orange pixel, this will be painted in the mask as white

//     // imshow("mask", mask); // show where orange pixels are located, then use this mask for further processing pass it for example to findNonZero() function in order to obtain the location of the pixels, etc...


//     cv::imshow("view", cv_ptr->image);
//     cv::waitKey(1);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
// }

void IRMarkerArrayCallback(const flightgoggles::IRMarkerArrayConstPtr& marker_array)
{
  int num_gates = 3;
  // char gates[11][10] = {"Gate10", "Gate21", "Gate2", "Gate13", "Gate9", "Gate14", "Gate1", "Gate22", "Gate15", "Gate23", "Gate6"};
  char gates[num_gates][10] = {"Gate9", "Gate23", "Gate6"};
  cv::Mat reprojection(768, 1024, CV_8UC3, cv::Scalar(0,0,0));
  double ulx, uly, urx, ury, lrx, lry, llx, lly;
  // Gate 10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6.
  for(int i = 0; i < num_gates; i++){
  
    for (flightgoggles::IRMarker marker : marker_array->markers){
      ROS_INFO("Landmark:%s, Marker:%s\n", marker.landmarkID.data.c_str(), marker.markerID.data.c_str());
      if(strcmp(marker.landmarkID.data.c_str(), gates[i]) == 0){
        if(strcmp(marker.markerID.data.c_str(), "1") == 0){ 
          // UR
          // cv::circle(reprojection, cv::Point(marker.x, marker.y), 5, cv::Scalar(0,0,255), -1);
          urx = marker.x;
          ury = marker.y;
        }else if(strcmp(marker.markerID.data.c_str(), "2") == 0){ 
          // UL
          // cv::circle(reprojection, cv::Point(marker.x, marker.y), 5, cv::Scalar(255,255,255), -1);
          ulx = marker.x;
          uly = marker.y;
        }else if(strcmp(marker.markerID.data.c_str(), "3") == 0){ 
          // LR
          // cv::circle(reprojection, cv::Point(marker.x, marker.y), 5, cv::Scalar(255,0,255), -1);
          lrx = marker.x;
          lry = marker.y;
        }else if(strcmp(marker.markerID.data.c_str(), "4") == 0){ 
          // LL
          // cv::circle(reprojection, cv::Point(marker.x, marker.y), 5, cv::Scalar(0,255,255), -1);
          llx = marker.x;
          lly = marker.y;
        }
      }
    }
    std::vector<cv::Point3f> objPts;
    objPts.push_back(cv::Point3f(0,0,0));
    objPts.push_back(cv::Point3f(1,0,0));
    objPts.push_back(cv::Point3f(0,1,0));
    objPts.push_back(cv::Point3f(1,1,0));

    std::vector<cv::Point2f> imgPts;
    imgPts.push_back(cv::Point2f(llx,lly));
    imgPts.push_back(cv::Point2f(lrx,lry));
    imgPts.push_back(cv::Point2f(ulx,uly));
    imgPts.push_back(cv::Point2f(urx,ury));

    double fx = 548.4088134765625, cx = 512.0, fy = 548.4088134765625, cy = 384.0;
    double camera_mat_vec[9] = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
    cv::Mat cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_mat_vec);

    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector  

    cv::solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Mat rotMatrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
    Rodrigues(rvec, rotMatrix);

    std::vector<cv::Point2f> projPts;
    std::vector<cv::Point3f> axisPts;
    axisPts.push_back(cv::Point3f(0, 0, 0)); // Origin
    axisPts.push_back(cv::Point3f(0, 1, 0)); // Y-Axis
    axisPts.push_back(cv::Point3f(0, 0, 1)); // Z-Axis
    axisPts.push_back(cv::Point3f(1, 0, 0)); // X-Axis
    cv::projectPoints(axisPts, rvec, tvec, cameraMatrix, distCoeffs, projPts);

      for(unsigned int i = 0; i < projPts.size(); ++i)
      {
        std::cout << "Image point: " << imgPts[i] << " Projected to " << projPts[i] << std::endl;
      }

    putText(reprojection, "UL", cv::Point(ulx, uly), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
    putText(reprojection, "UR", cv::Point(urx, ury), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
    putText(reprojection, "LR", cv::Point(lrx, lry), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
    putText(reprojection, "LL", cv::Point(llx, lly), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);

    cv::line(reprojection, cv::Point(ulx, uly), cv::Point(urx, ury), cv::Scalar(255,255,255), 1);
    cv::line(reprojection, cv::Point(urx, ury), cv::Point(lrx, lry), cv::Scalar(255,255,255), 1);
    cv::line(reprojection, cv::Point(lrx, lry), cv::Point(llx, lly), cv::Scalar(255,255,255), 1);
    cv::line(reprojection, cv::Point(llx, lly), cv::Point(ulx, uly), cv::Scalar(255,255,255), 1);

    cv::line(reprojection, cv::Point(projPts[0].x, projPts[0].y), cv::Point(projPts[1].x, projPts[1].y), cv::Scalar(128,255,255), 2);
    cv::line(reprojection, cv::Point(projPts[0].x, projPts[0].y), cv::Point(projPts[2].x, projPts[2].y), cv::Scalar(255,128,255), 2);
    cv::line(reprojection, cv::Point(projPts[0].x, projPts[0].y), cv::Point(projPts[3].x, projPts[3].y), cv::Scalar(255,255,128), 2);

  }
  imshow("Reprojection", reprojection);cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/bounding_box_camera/RGB", 1, imageCallback);

  ros::Subscriber ir_sub = nh.subscribe("/uav/camera/left/ir_beacons", 1, IRMarkerArrayCallback);

  ros::spin();
  cv::destroyAllWindows();
}