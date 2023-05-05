#include <image_saver.h>
#include <ros/ros.h>
int main(int argc, char** argv) {
  ros::init(argc, argv, "image_saver");
  ros::NodeHandle pnh("~");

  try {
    image_saver::ImageSaverROS single_node(pnh);
    //single_node.Run();
    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin(); 
    //single_node.End();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}