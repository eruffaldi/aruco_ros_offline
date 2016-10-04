// std msgs
// PoseStamped
// aruco?
// specify marker
// specify camera info
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define foreach BOOST_FOREACH

#include <ros/console.h>
#include <ros/assert.h>
#include <iostream>
#include <tf/transform_datatypes.h>

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
     //   ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
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

#if 0
void assign (geometry_msgs::PoseStamped t, const tf::Pose tf ,  const ros::Time &timestamp,const std::string &frame_id)
{
    tf::Stamped<tf::Pose> stf(tf,timestamp,frame_id);
    tf::poseStampedTFToMsg(t,stf);      
} 
#endif

void assign (geometry_msgs::TransformStamped & t, const tf::Transform tf, const ros::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
{
    tf::StampedTransform stf(tf,timestamp,frame_id,child_frame_id);
    tf::transformStampedTFToMsg(stf,t);      
} 

int main(int argc, char const *argv[])
{
    aruco::CameraParameters camParam;
    aruco::MarkerDetector mDetector;
    vector<aruco::Marker> markers;
    std::vector<std::string> topics;

    rosbag::Bag bag;
    rosbag::Bag bag2;
    if(argc == 1)
        return 0;
    bag.open(argv[1], rosbag::bagmode::Read);
    bag2.open(argv[2], rosbag::bagmode::Write);

    std::string jouttopic = "/joint_states";
    bool useRectifiedImages = false;
    topics.push_back(std::string("/kinect1/depth/camera_info"));
    topics.push_back(std::string("/kinect1/rgb/image/compressed"));
    topics.push_back(std::string(jouttopic));
    std::string outtopic = "/marker";
    int marker_id = 100;
    float marker_size = 0.20;


    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::CameraInfo ci;
    bool hasci = false;
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::JointState::ConstPtr j = m.instantiate<sensor_msgs::JointState>();
        if(j)
        {
            bag2.write(jouttopic,j->header.stamp,*j);
            continue;
        }

        sensor_msgs::CameraInfo::ConstPtr s = m.instantiate<sensor_msgs::CameraInfo>();
        if(s)
        {
            ci = *s;
            hasci = true;
            camParam = rosCameraInfo2ArucoCamParams(ci, useRectifiedImages);
            /*rightToLeft.setIdentity();
            rightToLeft.setOrigin(
                tf::Vector3(
                    -ci.P[3]/ci.P[0],
                    -ci.P[7]/ci.P[5],
                    0.0));
            */
                    continue;
        }

        sensor_msgs::CompressedImage::ConstPtr i = m.instantiate<sensor_msgs::CompressedImage>();
        if(i && hasci)
        {
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            cv_ptr->image = cv::imdecode(cv::Mat(i->data), cv::IMREAD_GRAYSCALE);
            //NOT NEEDED assignencoding(cv_ptr,message->format);

            auto inImage = cv_ptr->image; // cv::Mat
            markers.clear();
            //Ok, let's detect
            mDetector.detect(inImage, markers, camParam, marker_size, false);
            // publish ...
            bool done = false;
            for(auto & m: markers)
            {
              // only publishing the selected marker
              if(m.id == marker_id)
              {
                geometry_msgs::TransformStamped omsg;
                tf::Transform tt = arucoMarker2Tf(m);
                assign(omsg, tt, omsg.header.stamp, "marker","camera");
                omsg.header = i->header;

                bag2.write(outtopic,omsg.header.stamp,omsg);
                done = true;

              }
              else
              {
                std::cout << "skip marker " << m.id << std::endl;
              }
            }
            if(!done)
            {
                geometry_msgs::TransformStamped omsg;
                omsg.transform.translation.x = 0;
                omsg.transform.translation.y = 0;
                omsg.transform.translation.z = 0;
                omsg.transform.rotation.x = 0;
                omsg.transform.rotation.y = 0;
                omsg.transform.rotation.z = 0;
                omsg.transform.rotation.w = 0;
                omsg.child_frame_id = "";
                omsg.header = i->header;

                bag2.write(outtopic,omsg.header.stamp,omsg);

            }

        }   
    }
    bag2.close();
    bag.close();
    return 0;
}