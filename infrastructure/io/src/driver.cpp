#include <boost/thread/once.hpp>

#include <pcl2/io/driver.hpp>
#include <pcl2/io/openni_device_source_factory.hpp>

namespace pcl {
  namespace {
    std::vector<Driver::DriverFactory> all_driver_factories;
    boost::once_flag all_driver_factories_once = BOOST_ONCE_INIT;

    void initDriverFactories() {
//      all_device_source_factories.push_back(OpenNIDeviceSourceFactory::getInstance());
    }
  }

  const std::vector<Driver::DriverFactory> &
  Driver::getAllFactories() {
    boost::call_once(initDriverFactories, all_driver_factories_once);
    return all_driver_factories;
  }
}
