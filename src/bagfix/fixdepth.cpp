// std msgs
// PoseStamped
// specify marker
// specify camera info
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

#define foreach BOOST_FOREACH

#include <ros/console.h>
#include <ros/assert.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <fstream>

int main(int argc, char const *argv[])
{
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


    rosbag::View view(bag);

    sensor_msgs::CameraInfo ci;
    bool isfirst = true;
    bool hasci = false;
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::CompressedImage::ConstPtr j = m.instantiate<sensor_msgs::CompressedImage>();
        if(j && m.getTopic() == "/kinect1/depth/image/compressed")
        {
             cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

              // Copy message header
              cv_ptr->header = j->header;

            cv_ptr->image = cv::imdecode(cv::Mat(j->data),  cv::IMREAD_UNCHANGED);

            auto om = cv_ptr->toImageMsg();
            om->step *=1;
            om->encoding = "mono8";
            std::cout << "done " << m.getTopic() << " " << om->step << " " << om->data.size() << std::endl;
            if(isfirst)
            {
                isfirst = false;
                std::ofstream onf("x.jpg",std::ios::binary);
                onf.write((char*)&j->data[0],j->data.size());
            }

            

            bag2.write("/kinect1/depth/image_de",m.getTime(),om);
                    }
        else
        {
            bag2.write(m.getTopic(),m.getTime(),m, m.getConnectionHeader());
        

        }   
    }
    bag2.close();
    bag.close();
    return 0;
}