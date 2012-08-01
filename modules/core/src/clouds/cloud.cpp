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

#include "../precomp.hpp"


/////////////////////////////////////////////////////////////////////////////////////////////////
// CludaData

pcl::CloudData::CloudData() : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) {}
pcl::CloudData::CloudData(size_type sizeBytes) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true)
{ create(sizeBytes); }

pcl::CloudData::CloudData(size_type rows, size_type colsBytes)  : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true)
{ create(rows, colsBytes); }

pcl::CloudData::CloudData(size_type sizeBytes, void *data) : data_((unsigned char*)data), step_(sizeBytes), rows_(1), colsBytes_(sizeBytes), refcount_(0), dense_(true) {}
pcl::CloudData::CloudData(size_type rows, size_type colsBytes, void *data) : data_((unsigned char*)data), step_(colsBytes), rows_(rows), colsBytes_(colsBytes), refcount_(0), dense_(true) {}

pcl::CloudData::~CloudData() { release(); }


pcl::CloudData::CloudData(const CloudData& other) : data_(other.data_), step_(other.step_), rows_(other.rows_),
    colsBytes_(other.colsBytes_), refcount_(other.refcount_), dense_(other.dense_)
{
  if( refcount_ )
    __XADD__(refcount_, 1);
}

pcl::CloudData& pcl::CloudData::operator=(const CloudData& other)
{
  if( this != &other )
  {
    if( other.refcount_ )
      __XADD__(other.refcount_, 1);
    release();

    data_ = other.data_;
    step_ = other.step_;
    rows_ = other.rows_;
    colsBytes_ = other.colsBytes_;
    refcount_  = other.refcount_;
    dense_ = other.dense_;
  }
  return *this;
}

void pcl::CloudData::create(size_type sizeBytes) { create(1, sizeBytes); }
void pcl::CloudData::create(size_type rows, size_type colsBytes)
{
  if (rows  == rows_ && colsBytes == colsBytes_)
    return;

  if(rows * colsBytes > 0)
  {
    if( data_ )
      release();

     colsBytes_ = colsBytes;
     rows_ = rows;

     step_ = colsBytes_;

     data_ = operator new(rows_ * colsBytes_);
     refcount_ = new int;
     *refcount_ = 1;
   }
}

void pcl::CloudData::release()
{
  if( refcount_ && __XADD__(refcount_, -1) == 1 )
  {
    delete refcount_;
    operator delete(data_);
  }
  data_ = 0;
  step_ = 0;
  rows_ = 0;
  colsBytes_ = 0;
  refcount_ = 0;
}

void pcl::CloudData::swap(CloudData& other)
{
  std::swap(data_, other.data_);
  std::swap(step_, other.step_);
  std::swap(rows_, other.rows_);
  std::swap(colsBytes_, other.colsBytes_);
  std::swap(refcount_, other.refcount_);
  std::swap(dense_, other.dense_);
}

void pcl::CloudData::copyTo(CloudData& other) const
{
  if (empty())
    other.release();
  else
  {
    other.create(rows_, colsBytes_);

    const char *b = (const char*)data_;
    const char *e = b + rows_*colsBytes_;
    std::copy(b, e, (char*)other.data_);
  }
}

bool pcl::CloudData::empty() const { return !data_; }
pcl::CloudData::size_type pcl::CloudData::rows()      const { return  rows_; }
pcl::CloudData::size_type pcl::CloudData::colsBytes() const { return colsBytes_; }
pcl::CloudData::size_type pcl::CloudData::step()      const { return step_; }


/////////////////////////////////////////////////////////////////////////////////////////////////
// CloudSet

pcl::CloudSet::CloudSet()
{
  for(int i = 0; i < MAX_CLOUDS; ++i)
    storage_[i].type_ = ChannelKind::None;
}


pcl::CloudData pcl::CloudSet::create(size_type rows, size_type colsBytes, const std::string& name)
{
  int i = name_search(name);
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_.create(rows, colsBytes);
    storage_[i].type_ = ChannelKind::None;
    return storage_[i].data_;
  }

  i = empty_search();
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_.create(rows, colsBytes);
    storage_[i].name_ = name;
    storage_[i].type_ = ChannelKind::None;
  }
  else
      PCL_FatalError("Maximal supported clouds limit reached for CloudSet");
  return storage_[i].data_;
}

void pcl::CloudSet::set(const CloudData& data, const std::string& name)
{
  int i = name_search(name);

  if (i != MAX_CLOUDS)
  {
    storage_[i].data_ = data;
    return;
  }

  i = empty_search();
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_ = data;
    storage_[i].name_ = name;
    storage_[i].type_ = ChannelKind::None;
  }
  else
    PCL_FatalError("Maximal supported clouds limit reached for CloudSet");
}

void pcl::CloudSet::remove(const std::string& name)
{
  int i = name_search(name);
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_.release();
    storage_[i].name_.clear();
    storage_[i].type_ = ChannelKind::None;
  }
}

pcl::CloudData pcl::CloudSet::get(const std::string& name)
{
  int i = name_search(name);
  return i == MAX_CLOUDS ? CloudData() : storage_[i].data_;
}

const pcl::CloudData pcl::CloudSet::get(const std::string& name) const
{
  int i = name_search(name);
  return i == MAX_CLOUDS ? CloudData() : storage_[i].data_;
}

void pcl::CloudSet::set(const CloudData& cloud_data, int type)
{
  int i = type_search(type);
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_ = cloud_data;
    return;
  }

  i = empty_search();
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_ = cloud_data;
    storage_[i].type_ = type;
    storage_[i].name_.clear();
  }
  else
    PCL_FatalError("Maximal supported clouds limit reached for CloudSet");
}

void pcl::CloudSet::remove(int type)
{
  int i = type_search(type);
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_.release();
    storage_[i].name_.clear();
    storage_[i].type_ = ChannelKind::None;
  }
}

int pcl::CloudSet::name_search(const std::string& name) const
{
  int i = 0;
  for(; i < MAX_CLOUDS; ++i)
    if(storage_[i].name_ == name)
      break;
  return i;
}

int pcl::CloudSet::type_search(int type) const
{
  int i = 0;
  for(; i < MAX_CLOUDS; ++i)
    if(storage_[i].type_ == type)
      break;
  return i;
}

int pcl::CloudSet::empty_search() const
{
  int i = 0;
  for(; i < MAX_CLOUDS; ++i)
    if(storage_[i].name_.empty() && storage_[i].type_ == ChannelKind::None)
      break;
  return i;
}
