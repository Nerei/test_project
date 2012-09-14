#pragma once

#include <string>

#include <pcl2/io/source.hpp>

namespace pcl {

  struct DeviceSourceFactory {
    struct DeviceDescription {
      void * id;
      std::string name;
    };

    virtual const std::vector<DeviceDescription *> & getDeviceList() = 0;
    virtual Source * openDevice(void * device_id) = 0;

    static const std::vector<DeviceSourceFactory *> & getAll();

  private:
    DeviceSourceFactory(const DeviceSourceFactory &);
    DeviceSourceFactory & operator = (const DeviceSourceFactory &);
  };

}
