// https://github.com/jkammerl/compressed_depth_image_transport/blob/master/src/compressed_depth_publisher.cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
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
#include <tf2_msgs/TFMessage.h>

#define foreach BOOST_FOREACH

#include <ros/console.h>
#include <ros/assert.h>
#include <iostream>
#include <tf/transform_datatypes.h>
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1, INV_DEPTH
};

// Compression configuration
struct ConfigHeader
{
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};


cv_bridge::CvImagePtr decompressDepth(sensor_msgs::CompressedImage::ConstPtr message)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Assign image encoding
  std::string image_encoding = message->format.substr(0, message->format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message->data.size() > sizeof(ConfigHeader))
  {

    // Read compression type from stream
    ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message->data[0], sizeof(compressionConfig));

    // Get compressed image data
    const vector<uint8_t> imageData(message->data.begin() + sizeof(compressionConfig), message->data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32)
    {
      cv::Mat decompressed;
      try
      {
        // Decode image data
        decompressed = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = Mat(rows, cols, CV_32FC1);

        // Depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // Publish message to user callback
        return cv_ptr;
      }
    }
    else
    {
      // Decode raw image
      try
      {
        cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0))
        // Publish message to user callback
        return cv_ptr;
    }
  }    
  return 0;
}

int main(int argc, char const *argv[])
{

    rosbag::Bag bag;
    rosbag::Bag bag2;
    if(argc == 1)
        return 0;
    bag2.open("x.bag", rosbag::bagmode::Write);
    std::string sourcetopicC = "/kinect1/depth/image/compressedDepth";
    std::string outtopic = "/kinect1/depth/image";

    for(int k =1; k < argc; k++)
    {
        bag.open(argv[k], rosbag::bagmode::Read);

        rosbag::View view(bag); //, rosbag::TopicQuery(topics));


        foreach(rosbag::MessageInstance const m, view)
        {
            // color
            sensor_msgs::CompressedImage::ConstPtr i = m.instantiate<sensor_msgs::CompressedImage>();
            if(i && m.getTopic() == sourcetopicC)
            {
                cv_bridge::CvImagePtr r;

              if(i->format.find("compressedDepth") != std::string::npos )
              {
                r = decompressDepth(i);
                bag2.write(outtopic,m.getTime(),r->toImageMsg());
               }

            /*
              else
                r = decompressColor(i); // TODO add flag


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
                for(auto & ma: markers)
                {
                  // only publishing the selected marker
                  if(ma.id == marker_id)
                  {
                    geometry_msgs::TransformStamped omsg;
                    tf::Transform tt = arucoMarker2Tf(ma);
                    // TODO fix here below
                    assign(omsg, tt, fixtime ? m.getTime():omsg.header.stamp, "marker","camera");
                    omsg.header = i->header;
                    omsg.header.frame_id = sourceframe;
                    omsg.child_frame_id = targetframe;
                    if(fixtime)
    	                omsg.header.stamp = m.getTime();

                    bag2.write(outtopic,omsg.header.stamp,omsg);
                    done = true;

                    tf2_msgs::TFMessage tfx;
                    tfx.transforms.push_back(omsg);

                    bag2.write("/tf",omsg.header.stamp,tfx);

                  }
                  else
                  {
                    std::cout << "skip marker " << ma.id << std::endl;
                  }
                }
                if(!done && false) // not send
                {
                    geometry_msgs::TransformStamped omsg;
                    omsg.transform.translation.x = 0;
                    omsg.transform.translation.y = 0;
                    omsg.transform.translation.z = 0;
                    omsg.transform.rotation.x = 0;
                    omsg.transform.rotation.y = 0;
                    omsg.transform.rotation.z = 0;
                    omsg.transform.rotation.w = 0;
                    omsg.child_frame_id = targetframe;
                    omsg.header = i->header;
                    omsg.header.frame_id = sourceframe;
                	omsg.header.stamp = m.getTime();

                    bag2.write(outtopic,omsg.header.stamp,omsg);

                }
                */
                continue;
            }   
            bag2.write(m.getTopic(),m.getTime(),m, m.getConnectionHeader());


            }
        bag.close();
    }
    bag2.close();
    return 0;
}