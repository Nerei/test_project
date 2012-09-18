#pragma once

#include <pcl2/core/core.hpp>
#include <pcl2/io/simple_grabber.hpp>

namespace pcl {

  struct Grabber : SimpleGrabber {
    virtual bool getFrameTimeout(Cloud & target, unsigned long timeoutMs = 0) = 0;
    virtual bool isFrameAvailable() const = 0;
  };

}
