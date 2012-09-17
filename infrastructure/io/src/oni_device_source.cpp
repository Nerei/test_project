#include <boost/bind.hpp>

#include <pcl2/io/impl/oni_device_source.hpp>

namespace pcl {

  namespace {
    void oniNodeGetModes(const xn::MapGenerator & node, std::vector<XnMapOutputMode> & modes) {
      XnUInt32 count = node.GetSupportedMapOutputModesCount();
      modes.resize(count);

      if (count > 0) node.GetSupportedMapOutputModes(&modes.front(), count);
    }

    bool modesEqual(const XnMapOutputMode & m1, const XnMapOutputMode & m2) {
      return m1.nFPS == m2.nFPS && m1.nXRes == m2.nXRes && m1.nYRes == m2.nYRes;
    }
  }

  ONIDeviceSource::ONIDeviceSource(xn::Context & context, xn::NodeInfo & node_info)
    : oni_context_(context), depth_enabled_(false), color_enabled_(false)
  {
    context.CreateProductionTree(node_info, oni_device_);

    xn::Query query;
    query.AddNeededNode(node_info.GetInstanceName());

    context.CreateAnyProductionTree(XN_NODE_TYPE_DEPTH, &query, oni_depth_);
    context.CreateAnyProductionTree(XN_NODE_TYPE_IMAGE, &query, oni_image_);

    depth_available_ = oni_depth_.IsValid();
    color_available_ = oni_image_.IsValid();

    std::vector<XnMapOutputMode> depth_modes, color_modes;
    oniNodeGetModes(oni_depth_, depth_modes);
    oniNodeGetModes(oni_image_, color_modes);

    for (std::vector<XnMapOutputMode>::const_iterator mode_it = depth_modes.begin(); mode_it != depth_modes.end(); ++mode_it)
    {
      if (std::find_if(color_modes.begin(), color_modes.end(),
                       boost::bind(modesEqual, _1, *mode_it)) != color_modes.end()) {
        Mode source_mode;
        source_mode.frameRate = mode_it->nFPS;
        source_mode.pixelHeight = mode_it->nYRes;
        source_mode.pixelWidth = mode_it->nXRes;

        modes_.push_back(source_mode);
        modes_.back().id = &modes_.back();
      }
    }
  }

  const std::vector<Source::Mode> &
  ONIDeviceSource::getModes() const {
    return modes_;
  }

  void
  ONIDeviceSource::selectMode(void * mode_id) {
    Mode & source_mode = *static_cast<Mode *>(mode_id);
    XnMapOutputMode oni_mode = { source_mode.pixelWidth, source_mode.pixelHeight, source_mode.frameRate };

    if (depth_available_)
      oni_depth_.SetMapOutputMode(oni_mode);

    if (color_available_)
      oni_image_.SetMapOutputMode(oni_mode);
  }

  Grabber *
  ONIDeviceSource::start() {
    return NULL; // for now
  }

  bool
  ONIDeviceSource::isChannelAvailable(int kind) const {
    switch (kind) {
    case ChannelKind::RGB:
      return color_available_;

    case ChannelKind::Point3f:
      return depth_available_;

    default:
      return false;
    }
  }

  bool
  ONIDeviceSource::enableChannel(int kind, bool enabled) {
    switch (kind) {
    case ChannelKind::RGB:
      color_enabled_ = enabled;

    case ChannelKind::Point3f:
      depth_enabled_ = enabled;

    default:
      ; // ignore
    }
  }

}
