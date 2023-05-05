#include <image_saver.h>
#include <time.h>
#include <sensor_msgs/image_encodings.h>
#include <OpenImageIO/imageio.h>
#include <math.h>
#include <iostream>
#include <fstream>
using namespace OIIO;
namespace image_saver {


ImageSaverROS::ImageSaverROS(const ros::NodeHandle& pnh, const std::string& ns):
	pnh_(pnh, ns), 
	it_(pnh),
	xmpPacket(""),
	image_sub_(),
	xmpPacketFilePath(""),
	fileNameFormat(""),
	imageSeq(0)
{
	pnh_.param<double>("fix_time_tolerance", fixTimeTolerance, 0.2);
	// pnh_.param<double>("attitude_time_tolerance", attitudeTimeTolerance, 0.2);
	pnh_.param<std::string>("file_name_format", fileNameFormat, "%04d.tif");
	ROS_INFO_STREAM_NAMED(ros::this_node::getName(),"file name format is " << fileNameFormat);
	pixPitch=-1;
	image_sub_=it_.subscribe("image_raw", 1, &ImageSaverROS::imageCallback, this);
	fix_sub_=pnh_.subscribe<sensor_msgs::NavSatFix>("fix", 1, &ImageSaverROS::fixCallback, this);
	// attitude_sub_=pnh_.subscribe<geometry_msgs::QuaternionStamped>("/attitude", 1, &ImageSaverROS::attitudeCallback, this);
	//if (pnh.getParam("pix_pitch", pixPitch)==false)
	//{
	//    ROS_ERROR_NAMED(ros::this_node::getName(),"Can't get pix_pitch parameter.");
	//}
	loadxmp=false;
	if (pnh_.getParam("xmp_packet", xmpPacketFilePath))
	{
		loadxmpPackage();
		loadxmp=true;
	}
	imageCount=0;
}

/********** save image **************************/
void ImageSaverROS::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	imageSeq++;
	ros::Duration fixDiff=image->header.stamp-fix_.header.stamp;
	// ros::Duration attDiff=image->header.stamp-attitude_.header.stamp;
	bool fixTag=true;
	bool attitudeTag=true;
	if (abs(fixDiff.toSec())>fixTimeTolerance)
	{
		fixTag=false;
		ROS_WARN_NAMED(ros::this_node::getName(), "Timestampe between image and fix is larger than fix_time_tolerance");
	}
	// if (abs(attDiff.toSec())>attitudeTimeTolerance)
	// {
	// 	attitudeTag=false;
	// 	ROS_WARN_NAMED(ros::this_node::getName(), "Timestamp between image and attitude is larger than attitude_time_tolerance");
	// }
	
	time_t capture_time=image->header.stamp.toSec();
	struct tm * timeinfo;
	char timestr[20];
	char image_file_name[200] = {0};

	timeinfo = localtime(&capture_time);
	strftime(timestr, 20, "%Y:%m:%d %H:%M:%S", timeinfo);
	timestr[20] = '\0';

	if (sprintf(image_file_name, fileNameFormat.c_str(), imageSeq)<0)
	{
		ROS_WARN_NAMED(ros::this_node::getName(), "Can't format filename into user provided format. Using default formating.");
		sprintf(image_file_name, "%04d.tif", imageSeq);
	}

	ImageOutput* image_out = ImageOutput::create (image_file_name);
	if (! image_out)
	{
		ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Can't create image "<<image_file_name);
		return;
	}
	else
	{
		ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "Create image "<<image_file_name);
		unsigned int bitdepth=sensor_msgs::image_encodings::bitDepth(image->encoding);
		TypeDesc imageType=getImageType(image->encoding);
		ImageSpec spec (image->width, image->height, sensor_msgs::image_encodings::numChannels(image->encoding), imageType);

		
		spec.attribute("oiio:BitsPerSample", bitdepth);
		std::string attri;
		pnh_.param<std::string>("make", attri, "");
		spec.attribute("Make", attri);
		pnh_.param<std::string>("model", attri, "");
		spec.attribute("Model", attri);
		spec.attribute("tiff:compression", 1);
		spec.attribute("compression", "none");

		if (sensor_msgs::image_encodings::isColor(image->encoding))
			spec.attribute("tiff:PhotometricInterpretation", 2);
		else
		{
			spec.attribute("tiff:PhotometricInterpretation", 1);
		}
		spec.attribute("DateTime", timestr);
		pnh_.param<std::string>("copy_right", attri, "");
		spec.attribute("Copyright", attri);
		spec.attribute("Orientation", 1);

		// EXIF tags
		spec.attribute("Exif:DateTimeOriginal", timestr);
		spec.attribute("Exif:DateTimeDigitized", timestr);
		float focal_length=0;
		pnh_.param<float>("focal_length", focal_length, 0);
		spec.attribute("Exif:FocalLength", focal_length);
		if (loadxmp) spec.attribute("IPTC:MetadataDate", xmpPacket);
		if (fixTag)
		{
			// gps tags
			spec.attribute("GPS:LatitudeRef", (fix_.latitude>0)?"N":"S");
			float lat[3];
			lat[0]=floor(abs(fix_.latitude));
			lat[1]=floor((abs(fix_.latitude)-lat[0])*60);
			lat[2]=abs(fix_.latitude)-lat[0]-lat[1]/60;
			spec.attribute("GPS:Latitude", TypeDesc(TypeDesc::FLOAT, TypeDesc::SCALAR, TypeDesc::NOSEMANTICS, 3), lat);
			spec.attribute("GPS:LongitudeRef", (fix_.latitude>0)?"E":"W");
			float lng[3];
			lng[0]=floor(abs(fix_.longitude));
			lng[1]=floor((abs(fix_.longitude)-lng[0])*60);
			lng[2]=abs(fix_.longitude)-lng[0]-lng[1]/60;
			spec.attribute("GPS:Longitude", TypeDesc(TypeDesc::FLOAT, TypeDesc::SCALAR, TypeDesc::NOSEMANTICS, 3), lng);
			spec.attribute("GPS:AltitudeRef", (fix_.altitude>0)?"0":"1");
			spec.attribute("GPS:Altitude", (float)fix_.altitude);
			spec.attribute("GPS:MeasureMode", "3");
		}


		if (image_out->open(image_file_name, spec))
		{
			image_out->write_image(imageType, image->data.data());
			ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "image was written to " << image_file_name);
		}
		else
			ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Can't save image to disk"<<geterror());
		image_out->close();
	}
}

TypeDesc ImageSaverROS::getImageType(const std::string& encoding)
{
	if (encoding == "mono8"   	||
		encoding == "rgb8"        ||
		encoding == "rgba8"        ||
		encoding == "bgr8"      ||
		encoding == "bgra8"       ||
		encoding == "bayer_rggb8"||
		encoding == "bayer_bggr8" ||
		encoding == "bayer_gbrg8" ||
		encoding == "bayer_grbg8")
		return TypeDesc::UINT8;

	if (encoding == "mono16"      ||
		encoding == "rgb16"       ||
		encoding == "rgba16"        ||
		encoding == "bgr16"      ||
		encoding == "bgra16"       ||
		encoding == "bayer_rggb16" ||
		encoding == "bayer_bggr16" ||
		encoding == "bayer_gbrg16" ||
		encoding == "bayer_grbg16")
		{
			return TypeDesc::UINT16;
		}
	if (encoding == "32FC")
		return TypeDesc::FLOAT;
	if (encoding == "16UC")
		return TypeDesc::UINT16;
	if (encoding == "16SC")
		return TypeDesc::INT16;	
	if (encoding == "32SC")
		return TypeDesc::INT32;	
	if (encoding == "32UC")
		return TypeDesc::UINT32;
	if (encoding == "32FC")
		return TypeDesc::FLOAT;
	if (encoding == "64FC")
		return TypeDesc::DOUBLE;
	throw std::runtime_error("Unknown encoding " + encoding);
		return TypeDesc::UNKNOWN;
}


bool ImageSaverROS::loadxmpPackage()
{
	//ROS_DEBUG_STREAM_NAMED(ros::this_node::getName(), "reading xmp metadata from " << xmpPacketFilePath);

	std::ifstream file;
	std::stringstream strStream;
	file.open(xmpPacketFilePath,std::ios::in);
	if (file.is_open())
	{
		strStream << file.rdbuf();
		xmpPacket = strStream.str();
		ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "xmp metadata is\n"<<xmpPacket);
		return true;
	}
	else
	{
		ROS_WARN_STREAM_NAMED(ros::this_node::getName(), "Can't open file " << xmpPacketFilePath);
		return false;
	}
}
}
