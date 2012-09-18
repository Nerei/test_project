#pragma once

#include <pcl2/core/core.hpp>

namespace pcl {

  struct SimpleGrabber {
    enum EosReason { eosReasonNone, eosReasonEndOfFile, eosReasonDisconnect, eosReasonError };

    virtual ~SimpleGrabber() {}

    virtual bool getFrame(Cloud & target) = 0;
    virtual bool isEndOfStream() const = 0;
    virtual EosReason getEosReason() const = 0;

  protected:
    SimpleGrabber() {}
  private:
    SimpleGrabber(const SimpleGrabber &);
    SimpleGrabber & operator = (const SimpleGrabber &);
  };

}
