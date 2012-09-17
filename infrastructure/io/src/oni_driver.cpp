#include <XnCppWrapper.h>

#include <pcl2/io/oni_driver.hpp>

namespace pcl {
  class ONIDriver::Impl {
    xn::Context oniContext;
    std::vector<DeviceDescription> devices;
    std::vector<xn::NodeInfo> oniNodes;

  public:
    Impl() {
      oniContext.Init();
      refresh();
    }

    const std::vector<DeviceDescription> &
    getDeviceList() {
      return devices;
    }

    void
    refresh() {
      xn::NodeInfoList node_info_list;
      oniContext.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, node_info_list);
      oniNodes.clear();
      devices.clear();

      for (xn::NodeInfoList::Iterator node_it = node_info_list.Begin(); node_it != node_info_list.End(); ++node_it) {
        oniNodes.push_back(*node_it);

        DeviceDescription device;
        device.id = &oniNodes.back();
        device.name = oniNodes.back().GetInstanceName();
        devices.push_back(device);
      }
    }

    Source *
    openDevice(void * device_id) {
      return NULL; // for now
    }
  };

  ONIDriver::ONIDriver() {
    impl = new Impl();
  }

  ONIDriver::~ONIDriver() {
    delete impl;
  }

  const std::vector<Driver::DeviceDescription> &
  ONIDriver::getDeviceList() const {
    return impl->getDeviceList();
  }

  void
  ONIDriver::refresh() {
    return impl->refresh();
  }

  Source *
  ONIDriver::openDevice(void * device_id) {
    return impl->openDevice(device_id);
  }
}
