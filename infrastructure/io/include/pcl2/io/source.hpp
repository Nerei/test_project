#pragma once

#include <vector>

#include <pcl2/io/grabber.hpp>

namespace pcl {
  struct Source {
    struct Mode {
      void * id;
      int pixelWidth;
      int pixelHeight;
      int frameRate;
    };

    virtual ~Source() {}

    template <typename T>
    bool isChannelAvailable() const {
      return isChannelAvailable(channel_traits<T>::type);
    }

    template <typename T>
    void enableChannel(bool enabled) {
      enableChannel(channel_traits<T>::type, enabled);
    }

    virtual const std::vector<Mode> & getModes() const = 0;

    virtual void selectMode(void * mode_id) = 0;

    virtual Grabber * start() = 0;

  protected:
    Source() {}
    virtual bool isChannelAvailable(int kind) const = 0;
    virtual bool enableChannel(int kind, bool enabled) = 0;

  private:
    Source(const Source &);
    Source & operator = (const Source &);
  };
}
