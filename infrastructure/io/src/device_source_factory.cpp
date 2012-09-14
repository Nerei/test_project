#include <pcl2/io/device_source_factory.hpp>

namespace pcl {
  namespace {
    std::vector<DeviceSourceFactory *> all_device_source_factories;
  }

  const std::vector<DeviceSourceFactory *> & DeviceSourceFactory::getAll() {
    return all_device_source_factories;
  }
}
