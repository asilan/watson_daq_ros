#include "image_saver.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace image_saver {

class SingleNodelet : public nodelet::Nodelet {
 public:
  SingleNodelet() = default;
  ~SingleNodelet() =default;
  virtual void onInit() {
    try {
      single_node_.reset(new ImageSaverROS(getMTPrivateNodeHandle()));
    } catch (const std::exception &e) {
      NODELET_ERROR("%s: %s", getNodeHandle().getNamespace().c_str(),
                    e.what());
    }
  }

 private:
  std::unique_ptr<ImageSaverROS> single_node_;
};

PLUGINLIB_EXPORT_CLASS(image_saver::SingleNodelet, nodelet::Nodelet)

}  // namespace tau2