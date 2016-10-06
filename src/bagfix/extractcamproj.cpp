// https://github.com/jkammerl/compressed_depth_image_transport/blob/master/src/compressed_depth_publisher.cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <bagfix/Plane.h>

#define foreach BOOST_FOREACH

#include <ros/console.h>
#include <ros/assert.h>
#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;



int main(int argc, char const *argv[])
{

    rosbag::Bag bag;
    if(argc == 1)
        return 0;
    std::string planetopic = "/plane";
    std::string colortopic = "/kinect1/rgb/image/compressed";

    mkdir("images",0700);
    mkdir("plane",0700);
    ros::Time last(0,0);
        int id =0;
    bool seen = false;
    for(int k =1; k < argc; k++)
    {
        bag.open(argv[k], rosbag::bagmode::Read);

        rosbag::View view(bag); //, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            // color
            {
              sensor_msgs::CompressedImage::ConstPtr i = m.instantiate<sensor_msgs::CompressedImage>();
              if(i && m.getTopic() == colortopic)
              {
                if(seen)
                {
                  seen = false;

                 char buf[128];
                 sprintf(buf,"images/%04d_color.jpg",id);
                 std::ofstream onf(buf);
                 onf.write((char*)&i->data[0],i->data.size());
                }
              }
              }
            bagfix::Plane::ConstPtr i = m.instantiate<bagfix::Plane>();
            if(i && m.getTopic() == planetopic)
            {
                   id++;
                    std::cout << "new Plane " << id << " after " << (m.getTime()-last) << std::endl;
                    last = m.getTime();
                    seen = true;
               char buf[128];
               sprintf(buf,"plane/%04d_color.yaml",id);
               // YAML
               //ros::package::getPath("planar_segmentator")+"/data/"+file_name
                std::string config_path = buf ;
                std::ofstream ofs(config_path.c_str());
                cv::FileStorage fs(config_path, cv::FileStorage::WRITE);

                cv::Vec4f planes_yaml(i->a,i->b,i->c,i->d);

                fs << "plane_coefficients";
                fs << "[" << planes_yaml << "]" ;
                fs.release();   
            }
        }
        bag.close();
    }
    return 0;
}