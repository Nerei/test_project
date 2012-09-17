#pragma once

#include <string>

#include <pcl2/io/source.hpp>

namespace pcl {

  struct Driver {
    struct DeviceDescription {
      void * id;
      std::string name;
    };

    virtual ~Driver() {}

    virtual const std::vector<DeviceDescription> & getDeviceList() const = 0;
    virtual Source * openDevice(void * device_id) = 0;
    virtual void refresh() = 0;

    typedef Driver * (* DriverFactory)();
    static const std::vector<DriverFactory> & getAllFactories();

  protected:
    Driver() {}

  private:
    Driver(const Driver &);
    Driver & operator = (const Driver &);
  };

}
