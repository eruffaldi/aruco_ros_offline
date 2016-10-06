

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <bagfix/Markers2D.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace aruco;

  cv::Point2f getCenter(MarkerInfo & mi)
  {
    cv::Point2f cent(0,0);
    for(size_t i=0;i<mi.size();i++){
      cent.x+=mi[i].x;
      cent.y+=mi[i].y;
    }
    cent.x/=float(mi.size());
    cent.y/=float(mi.size());
    return cent;
  }



aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                bool useRectifiedParameters)
{
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if ( useRectifiedParameters )
    {
      cameraMatrix.setTo(0);
      cameraMatrix.at<double>(0,0) = cam_info.P[0];   cameraMatrix.at<double>(0,1) = cam_info.P[1];   cameraMatrix.at<double>(0,2) = cam_info.P[2];
      cameraMatrix.at<double>(1,0) = cam_info.P[4];   cameraMatrix.at<double>(1,1) = cam_info.P[5];   cameraMatrix.at<double>(1,2) = cam_info.P[6];
      cameraMatrix.at<double>(2,0) = cam_info.P[8];   cameraMatrix.at<double>(2,1) = cam_info.P[9];   cameraMatrix.at<double>(2,2) = cam_info.P[10];

      for(int i=0; i<4; ++i)
        distorsionCoeff.at<double>(i, 0) = 0;
    }
    else
    {
      for(int i=0; i<9; ++i)
        cameraMatrix.at<double>(i%3, i-(i%3)*3) = cam_info.K[i];

      if(cam_info.D.size() == 4)
      {
        for(int i=0; i<4; ++i)
          distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
      }
      else
      {
        ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
        for(int i=0; i<4; ++i)
          distorsionCoeff.at<double>(i, 0) = 0;
      }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf::Transform arucoMarker2Tf(const aruco::Marker &marker)
{
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Mat Rvec64;
    marker.Rvec.convertTo(Rvec64, CV_64FC1);
    cv::Rodrigues(Rvec64, rot);
    cv::Mat tran64;
    marker.Tvec.convertTo(tran64, CV_64FC1);

    cv::Mat rotate_to_ros(3, 3, CV_64FC1);
    // -1 0 0
    // 0 0 1
    // 0 1 0
    rotate_to_ros.at<double>(0,0) = -1.0;
    rotate_to_ros.at<double>(0,1) = 0.0;
    rotate_to_ros.at<double>(0,2) = 0.0;
    rotate_to_ros.at<double>(1,0) = 0.0;
    rotate_to_ros.at<double>(1,1) = 0.0;
    rotate_to_ros.at<double>(1,2) = 1.0;
    rotate_to_ros.at<double>(2,0) = 0.0;
    rotate_to_ros.at<double>(2,1) = 1.0;
    rotate_to_ros.at<double>(2,2) = 0.0;
    rot = rot*rotate_to_ros.t();

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    tf::Vector3 tf_orig(tran64.at<double>(0,0), tran64.at<double>(1,0), tran64.at<double>(2,0));


    return tf::Transform(tf_rot, tf_orig);
}

tf::Transform getTf(const cv::Mat &Rvec, const cv::Mat &Tvec)
{
  cv::Mat rot(3, 3, CV_32FC1);
  cv::Rodrigues(Rvec, rot);

  cv::Mat rotate_to_sys(3, 3, CV_32FC1);
  /**
  /* Fixed the rotation to meet the ROS system
  /* Doing a basic rotation around X with theta=PI
  /* By Sahloul
  /* See http://en.wikipedia.org/wiki/Rotation_matrix for details
  */

  //  1 0 0
  //  0 -1  0
  //  0 0 -1
  rotate_to_sys.at<float>(0,0) = 1.0;
  rotate_to_sys.at<float>(0,1) = 0.0;
  rotate_to_sys.at<float>(0,2) = 0.0;
  rotate_to_sys.at<float>(1,0) = 0.0;
  rotate_to_sys.at<float>(1,1) = -1.0;
  rotate_to_sys.at<float>(1,2) = 0.0;
  rotate_to_sys.at<float>(2,0) = 0.0;
  rotate_to_sys.at<float>(2,1) = 0.0;
  rotate_to_sys.at<float>(2,2) = -1.0;
  rot = rot*rotate_to_sys.t();

  tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
    rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
    rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

  tf::Vector3 tf_orig(Tvec.at<float>(0,0), Tvec.at<float>(1,0), Tvec.at<float>(2,0));

  return tf::Transform(tf_rot, tf_orig);
}

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub; 
  ros::Publisher position_pub;
  ros::Publisher marker_pub; //rviz visualization marker
  ros::Publisher pixel_pub;
  ros::Publisher board_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  std::string image;
  std::string board_config;
  std::string board_frame;
  aruco::BoardConfiguration the_board_config;
  aruco::BoardDetector the_board_detector;
  aruco::Board the_board_detected;
  std::map<int,MarkerInfo> id2boardpos;


  double marker_size;
  int marker_id;

  ros::NodeHandle pnh;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

public:
  ArucoSimple()
    : cam_info_received(false),      
      pnh("~"),
      it(nh)
  {

    pnh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
    pnh.param<std::string>("image", image, "input.png");

    ROS_INFO_STREAM("board_config: " <<  board_config );
    ROS_INFO_STREAM("image: " <<  image );

    int marker_size = 0.1;
    auto mat = cv::imread(image);
    markers.clear();
    mDetector.detect(mat, markers, camParam, marker_size, false);
    aruco::BoardConfiguration the_board_config;
    the_board_config.mInfoType = BoardConfiguration::PIX;
    for(int i = 0; i < markers.size(); i++)
    {
        aruco::MarkerInfo mi(markers[i].id); // from markers
        for(int j = 0; j < markers[i].size(); j++)
          mi.push_back(cv::Point3f(markers[i][j].x,markers[i][j].y,0));
        the_board_config.push_back(mi);
    }
    the_board_config.saveToFile(board_config);
    exit(0);
}
};

int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
