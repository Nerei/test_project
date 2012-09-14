#pragma once

#include <pcl2/core/core.hpp>

namespace pcl {

  struct Grabber {
    enum EosReason { eosReasonNone, eosReasonEndOfFile, eosReasonDisconnect, eosReasonError };

    virtual ~Grabber() {}

    virtual bool getFrame(Cloud & target) = 0;
    virtual bool getFrameTimeout(Cloud & target, unsigned long timeoutMs = 0) = 0;
    virtual bool isFrameAvailable() const = 0;
    virtual bool isEndOfStream() const = 0;
    virtual EosReason getEosReason() const = 0;

  private:
    Grabber(const Grabber &);
    Grabber & operator = (const Grabber &);
  };

}
