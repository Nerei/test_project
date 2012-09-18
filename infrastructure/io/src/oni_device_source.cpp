#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/thread.hpp>

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

  class ONIDeviceSource::Grabber : public ::pcl::Grabber {
    ONIDeviceSource * source_;
    bool eos_;
    EosReason eos_reason_;
    XnCallbackHandle oni_cb_depth_, oni_cb_image_;

    bool new_data_;
    mutable boost::mutex mutex_new_data_;
    boost::condition_variable cond_new_data_;

    boost::container::flat_set<XnNodeHandle> updated_nodes_;

    static void oniDeviceSourceGrabberHandleNewData(xn::ProductionNode & node, void * p_cookie) {
      Grabber * p_grabber = static_cast<Grabber *>(p_cookie);
      p_grabber->handleNewData(node);
    }

    void handleNewData(xn::ProductionNode & node) {
      boost::unique_lock<boost::mutex> lock_new_data(mutex_new_data_);
      updated_nodes_.insert(node.GetHandle());

      if (updated_nodes_.size() == source_->color_enabled_ + source_->depth_enabled_) {
        new_data_ = true;
        cond_new_data_.notify_one();
        updated_nodes_.clear();

        if (source_->depth_enabled_)
          source_->oni_depth_.WaitAndUpdateData();

        if (source_->color_enabled_)
          source_->oni_image_.WaitAndUpdateData();
      }
    }

    bool saveCurrentFrameToCloud(Cloud & target) {
      if (source_->color_enabled_) {
        xn::ImageMetaData image_md;
        source_->oni_image_.GetMetaData(image_md);

        if (image_md.XRes() != source_->selected_mode_.pixelWidth || image_md.YRes() != source_->selected_mode_.pixelHeight)
        {
          eos_ = true;
          eos_reason_ = eosReasonError;
          return false;
        }

        const XnRGB24Pixel * image_pixels = image_md.RGB24Data();

        Channel<RGB> color_channel = target.create<RGB>(image_md.YRes(), image_md.XRes());

        for (int row = 0; row < image_md.YRes(); ++row)
          for (int col = 0; col < image_md.XRes(); ++col)
          {
            color_channel.ptr(row)[col].r = image_pixels->nRed;
            color_channel.ptr(row)[col].g = image_pixels->nGreen;
            color_channel.ptr(row)[col].b = image_pixels->nBlue;
            ++image_pixels;
          }
      }

      if (source_->depth_enabled_) {
        xn::DepthMetaData depth_md;
        source_->oni_depth_.GetMetaData(depth_md);

        if (depth_md.XRes() != source_->selected_mode_.pixelWidth || depth_md.YRes() != source_->selected_mode_.pixelHeight)
        {
          eos_ = true;
          eos_reason_ = eosReasonError;
          return false;
        }

        const XnDepthPixel * depth_pixels = depth_md.Data();

        Channel<Point3f> depth_channel = target.create<Point3f>(depth_md.YRes(), depth_md.XRes());

        for (int row = 0; row < depth_md.YRes(); ++row)
          for (int col = 0; col < depth_md.XRes(); ++col)
          {
            depth_channel.ptr(row)[col].x = col;
            depth_channel.ptr(row)[col].y = row;
            depth_channel.ptr(row)[col].z = *depth_pixels;
            ++depth_pixels;
          }
      }

      return true;
    }

  public:
    explicit Grabber(ONIDeviceSource * source) : source_(source), eos_(false), eos_reason_(eosReasonNone), new_data_(false)
    {
      if (source_->depth_enabled_) {
        source_->oni_depth_.StartGenerating();
        source_->oni_depth_.RegisterToNewDataAvailable(oniDeviceSourceGrabberHandleNewData, this, oni_cb_depth_);
      }

      if (source_->color_enabled_) {
        source_->oni_image_.StartGenerating();
        source_->oni_image_.RegisterToNewDataAvailable(oniDeviceSourceGrabberHandleNewData, this, oni_cb_image_);
      }
    }

    virtual ~Grabber() {
      if (source_->depth_enabled_) {
        source_->oni_depth_.UnregisterFromNewDataAvailable(oni_cb_depth_);
        source_->oni_depth_.StopGenerating();
      }

      if (source_->color_enabled_) {
        source_->oni_image_.UnregisterFromNewDataAvailable(oni_cb_image_);
        source_->oni_image_.StopGenerating();
      }
    }

    virtual bool getFrame(Cloud & target) {
      boost::unique_lock<boost::mutex> lock_new_data(mutex_new_data_);

      while (!new_data_)
        cond_new_data_.wait(lock_new_data);

      new_data_ = false;

      return saveCurrentFrameToCloud(target);
    }

    virtual bool isEndOfStream() const {
      return eos_;
    }

    virtual EosReason getEosReason() const {
      return eos_reason_;
    }

    virtual bool getFrameTimeout(Cloud & target, unsigned long timeoutMs = 0) {
      boost::unique_lock<boost::mutex> lock_new_data(mutex_new_data_);

      boost::chrono::steady_clock::time_point wait_end = boost::chrono::steady_clock::now() + boost::chrono::milliseconds(timeoutMs);

      while (!new_data_)
        if (cond_new_data_.wait_until(lock_new_data, wait_end) == boost::cv_status::timeout) return false;

      new_data_ = false;

      return saveCurrentFrameToCloud(target);
    }

    virtual bool isFrameAvailable() const {
      boost::unique_lock<boost::mutex> lock_new_data(mutex_new_data_);
      return new_data_;
    }

  };

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
    selected_mode_ = *static_cast<Mode *>(mode_id);
    XnMapOutputMode oni_mode = { selected_mode_.pixelWidth, selected_mode_.pixelHeight, selected_mode_.frameRate };

    if (depth_available_)
      oni_depth_.SetMapOutputMode(oni_mode);

    if (color_available_)
      oni_image_.SetMapOutputMode(oni_mode);
  }

  Grabber *
  ONIDeviceSource::start() {
    return new Grabber(this);
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
