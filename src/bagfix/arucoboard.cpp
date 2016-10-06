

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

    std::string refinementMethod;
    nh.param<std::string>("corner_refinement", refinementMethod, "LINES");
    if ( refinementMethod == "SUBPIX" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
    else if ( refinementMethod == "HARRIS" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    else if ( refinementMethod == "NONE" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE); 
    else      
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES); 

    //Print parameters of aruco marker detector:
    ROS_INFO_STREAM("Corner refinement method: " << mDetector.getCornerRefinementMethod());
    ROS_INFO_STREAM("Threshold method: " << mDetector.getThresholdMethod());
    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: " << " th1: " << th1 << " th2: " << th2);
    float mins, maxs;
    mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << mDetector.getDesiredSpeed());
    

    pnh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
    ROS_INFO_STREAM("board_config: " <<  board_config );
    pnh.param<std::string>("board_frame", board_frame, "");


      the_board_config.readFromFile(board_config.c_str());
      std::vector<int> ids;
      the_board_config.getIdList (ids);
      for(auto id : ids)
      {
        id2boardpos[id] = the_board_config.getMarkerInfo(id);
      }



    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);
    board_pub  = nh.advertise<bagfix::Markers2D>("board", 10);

    pnh.param<double>("marker_size", marker_size, 0.05);
    pnh.param<int>("marker_id", marker_id, 300);
    pnh.param<std::string>("reference_frame", reference_frame, "");
    pnh.param<std::string>("camera_frame", camera_frame, "");
    pnh.param<std::string>("marker_frame", marker_frame, "");
    pnh.param<bool>("image_is_rectified", useRectifiedImages, true);

      ROS_INFO("ARUCO Board node started with marker size of %f m and board configuration: %s and markers: %d",
           marker_size, board_config.c_str(),id2boardpos.size());

    //ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
             marker_size, marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());
  }

  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(0.5),
                                       ros::Duration(0.01),
                                       &errMsg)
         )
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame,
                                     ros::Time(0),                  //get latest available
                                     transform);
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }


  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    std::cout << "IMAGE!!!!" << std::endl;
    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time curr_stamp(ros::Time::now());
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        //detection results will go into "markers"
        markers.clear();
        //Ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        float probDetect=the_board_detector.detect(markers, the_board_config, the_board_detected, camParam, marker_size);
        bool found = false;
        if (probDetect > 0.0)
        {
          found = true;
            tf::Transform transform = getTf(the_board_detected.Rvec, the_board_detected.Tvec);
            tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, board_frame);
                    
            br.sendTransform(stampedTransform);


        }
        int imagepubn = image_pub.getNumSubscribers();
        int markerpubn = marker_pub.getNumSubscribers();
        int pixel_pubn = pixel_pub.getNumSubscribers();

        // ALL MARKERS
        bagfix::Markers2D markers2D; // publish this
        markers2D.found = found;

        //for each marker, draw info and its boundaries in the image
        for(size_t i=0; i<markers.size(); ++i)
        {
          // all markers
          //if(markers[i].id == marker_id)
          auto it = id2boardpos.find(markers[i].id);
          if(it != id2boardpos.end())
          {
              bagfix::Marker2D ma;
              ma.x = markers[i].getCenter().x;
              ma.y = markers[i].getCenter().y;
              auto c = getCenter(it->second);
              ma.ox = c.x;
              ma.oy = c.y;
              ma.id = it->first;
              markers2D.markers.push_back(ma);
              std::cout << "emit board marker " << it->first << std::endl;
          }
          {
            std::ostringstream ons;
            ons << marker_frame << "_" << markers[i].id;
            tf::Transform transform = arucoMarker2Tf(markers[i]);
            tf::StampedTransform cameraToReference;
            cameraToReference.setIdentity();
              std::cout << "emit  marker " << markers[i].id << " as " << marker_frame << std::endl;

            if ( reference_frame != camera_frame )
            {
              getTransform(reference_frame,
                           camera_frame,
                           cameraToReference);
            }

            transform = 
              static_cast<tf::Transform>(cameraToReference) 
              * static_cast<tf::Transform>(rightToLeft) 
              * transform;

            tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                  reference_frame, ons.str());
            br.sendTransform(stampedTransform);
            geometry_msgs::PoseStamped poseMsg;
            tf::poseTFToMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = curr_stamp;
            pose_pub.publish(poseMsg);

            geometry_msgs::TransformStamped transformMsg;
            tf::transformStampedTFToMsg(stampedTransform, transformMsg);
            transform_pub.publish(transformMsg);

            /*

            if(pixel_pubn)
            {
              geometry_msgs::PointStamped pixelMsg;
              pixelMsg.header = transformMsg.header;
              pixelMsg.point.x = markers[i].getCenter().x;
              pixelMsg.point.y = markers[i].getCenter().y;
              pixelMsg.point.z = 0;
              board_pub.publish(pixelMsg);
            }
            */

            //Publish rviz marker representing the ArUco marker patch
            if(markerpubn)
            {
            visualization_msgs::Marker visMarker;
            visMarker.header = transformMsg.header;
            visMarker.pose = poseMsg.pose;
            visMarker.id = 1;
            visMarker.type   = visualization_msgs::Marker::CUBE;
            visMarker.action = visualization_msgs::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = marker_size;
            visMarker.scale.y = 0.001;
            visMarker.scale.z = marker_size;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = ros::Duration(3.0);
            marker_pub.publish(visMarker);
          }

          }
          // but drawing all the detected markers
          if(imagepubn)
          markers[i].draw(inImage,cv::Scalar(0,0,255),2);
        }
        board_pub.publish(markers2D);


        if(imagepubn > 0)
        {
                  //draw a 3d cube in each marker if there is 3d info
                if(camParam.isValid() && marker_size!=-1)
                {
                  for(size_t i=0; i<markers.size(); ++i)
                  {
                    CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
                  }
                }


          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
