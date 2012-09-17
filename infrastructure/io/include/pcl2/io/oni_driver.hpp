#pragma once

#include <pcl2/io/driver.hpp>

namespace pcl {

  class ONIDriver : public Driver {
    class Impl;
    Impl * impl;

  public:
    ONIDriver();
    virtual ~ONIDriver();

    virtual const std::vector<DeviceDescription> & getDeviceList() const;
    virtual void refresh();
    virtual Source * openDevice(void * device_id);
  };

}
