#include <boost/thread/once.hpp>

#include <pcl2/io/driver.hpp>
#include <pcl2/io/oni_driver.hpp>

namespace pcl {
  namespace {
    std::vector<Driver::DriverFactory> all_driver_factories;
    boost::once_flag all_driver_factories_once = BOOST_ONCE_INIT;

    Driver * createONIDriver() {
      return new ONIDriver();
    }

    void initDriverFactories() {
      all_driver_factories.push_back(createONIDriver);
    }
  }

  const std::vector<Driver::DriverFactory> &
  Driver::getAllFactories() {
    boost::call_once(initDriverFactories, all_driver_factories_once);
    return all_driver_factories;
  }
}
