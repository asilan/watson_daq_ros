#ifndef IMAGE_SAVER_H_
#define IMAGE_SAVER_H_

//ros header files
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <OpenImageIO/imageio.h>

using namespace OIIO;

namespace image_saver {
class ImageSaverROS
{
public:
    ImageSaverROS(const ros::NodeHandle& pnh, const std::string& ns = std::string());
    ImageSaverROS() = delete;
    ImageSaverROS(const ImageSaverROS&) = delete;
    ImageSaverROS& operator=(const ImageSaverROS&) = delete;
    void fixCallback(const sensor_msgs::NavSatFixConstPtr& fix);
    // void attitudeCallback(geometry_msgs::QuaternionStamped attitude)
    // {
    //     ROS_DEBUG_NAMED(ros::this_node::getName(),"Run attitude callback");
    //     attitude_=attitude;
    // }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    double getPixPitch() {return pixPitch;}
    bool loadxmpPackage();
    TypeDesc getImageType(const std::string& encoding);
    int getChannelOrder(const std::string& encoding);
private:
	ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber fix_sub_;
    // ros::Subscriber attitude_sub_;
    sensor_msgs::NavSatFix fix_;
    // geometry_msgs::QuaternionStamped attitude_;
    double fixTimeTolerance;
    double attitudeTimeTolerance;
    std::string fileNameFormat;
    std::string rootFolder;
    int imageCount;
    double pixPitch;
    std::string xmpPacketFilePath;
    std::string xmpPacket;
    int imageSeq;
    bool loadxmp;
bool fixReceived;
};
}


#endif  // IMAGE_SAVER_H
