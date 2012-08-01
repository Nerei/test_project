/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl2/core/types.hpp>
#include <pcl2/core/clouds/cloud.hpp>
#include <vector>
#include <string>

namespace pcl
{
  class PCL_EXPORTS In
  {
  public:
    typedef size_t size_type;

    In();

    template<typename T> In(const T& data);
    template<typename T> In(const Cloud<T>& cloud);
    template<typename T> In(const std::vector<T>& cloud);

    template<typename T>
    const Cloud<T>  get(const std::string& name) const;
    const CloudData get(const std::string& name) const;

    template<typename T>
    const Cloud<T>  get() const;
    const CloudData get() const;

    int type() const;
  protected:

    CloudData from_vector(size_t size_bytes = 0, bool allocate = false) const;

    struct Kind
    {
      enum { None, Cloud, CloudSet, StdVector };
    };

    int kind_;
    int type_;
    void* obj_;
  };

  class PCL_EXPORTS Out : public In
  {
  public:
    Out();
    template<typename T> Out(const T& data);
    template<typename T> Out(const Cloud<T>& cloud);
    template<typename T> Out(const std::vector<T>& cloud);

    template<typename T>
    Cloud<T> create(size_type rows, size_type cols);
    CloudData create(size_type rows, size_type colsBytes, const std::string& name);
    CloudData create(size_type rows, size_type colsBytes);

  };

  template<> PCL_EXPORTS In::In(const CloudData& cloud_data);
  template<> PCL_EXPORTS In::In(const CloudSet& cloud_set);
  template<> PCL_EXPORTS pcl::Out::Out(const CloudData& cloud_data);
  template<> PCL_EXPORTS pcl::Out::Out(const CloudSet& cloud_set);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Implementation


template<typename T> inline pcl::In::In(const Cloud<T>& cloud) : kind_(Kind::Cloud), type_(channel_traits<T>::type), obj_((void)*&cloud) {}
template<typename T> inline pcl::In::In(const std::vector<T>& cloud)  : kind_(Kind::StdVector), type_(channel_traits<T>::type), obj_((void)*&cloud) {}

template<typename T> inline const pcl::Cloud<T> pcl::In::get(const std::string& name) const { return Cloud<T>(get(name)); }

template<typename T> inline const pcl::Cloud<T> pcl::In::get() const
{
  if (kind_ == Kind::CloudSet)
    return reinterpret_cast<const CloudSet*>(obj_)->get<T>();
  else
    return (type_ == channel_traits<T>::type) ? Cloud<T>(get()) : Cloud<T>();
}

template<typename T> inline pcl::Out::Out(const Cloud<T>& cloud) : In(cloud) {}
template<typename T> inline pcl::Out::Out(const std::vector<T>& cloud)  : In(cloud) {}

template<typename T> inline pcl::Cloud<T> pcl::Out::create(size_type rows, size_type cols)
{
  if (kind_ == Kind::CloudSet)
  {
    CloudSet& cloud_set = *reinterpret_cast<CloudSet*>(obj_);
    return cloud_set.create<T>(rows, cols);
  }

  PCL_Assert(channel_traits<T>::type == type_);

  if (kind_ == Kind::Cloud)
  {
    Cloud<T>& cloud = *reinterpret_cast<Cloud<T>*>(obj_);
    return cloud.create(rows, cols), cloud;
  }
  else /* Kind::StdVector */
    return Cloud<T>(from_vector(cols * rows*sizeof(T), true));
}

