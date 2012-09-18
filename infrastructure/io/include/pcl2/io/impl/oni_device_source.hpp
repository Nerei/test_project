#pragma once

#include <XnCppWrapper.h>

#include <pcl2/io/source.hpp>

namespace pcl {

  class ONIDeviceSource : public Source {
    xn::Context oni_context_;
    xn::Device oni_device_;
    xn::DepthGenerator oni_depth_;
    xn::ImageGenerator oni_image_;

    std::vector<Mode> modes_;
    Mode selected_mode_;

    bool depth_available_, depth_enabled_;
    bool color_available_, color_enabled_;

    class Grabber;

  public:
    ONIDeviceSource(xn::Context & context, xn::NodeInfo & node_info);

    virtual const std::vector<Mode> & getModes() const;
    virtual void selectMode(void * mode_id);
    virtual ::pcl::Grabber * start();

  protected:
    virtual bool isChannelAvailable(int kind) const;
    virtual bool enableChannel(int kind, bool enabled);
  };

}
