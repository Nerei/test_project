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

#include <pcl2/core/pcl_exports.hpp>
#include <pcl2/core/types.hpp>
#include <pcl2/core/errors.hpp>
#include <string>
#include <vector>


namespace pcl
{
   //////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ChannelData class
    * \note BLOB container class with reference counting.
    * \author Anatoly Baksheev
    */
  class PCL_EXPORTS ChannelData
  {
  public:

    /** \brief Size type */
    typedef size_t size_type;

    /** \brief Empty constructor */
    ChannelData();

    /** \brief Destructor */
    ~ChannelData();

    /** \brief Allocates internal buffer */
    ChannelData(size_type sizeBytes);

    /** \brief Allocates internal buffer */
    ChannelData(size_type rows, size_type colsBytes);

    /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
    ChannelData(size_type sizeBytes, void *data);

    /** \ChannelData Initializes with user allocated buffer. Reference counting is disabled in this case */
    ChannelData(size_type rows, size_type colsBytes, void *data);

    /** \brief Copy constructor. Just increments reference counter */
    ChannelData(const ChannelData& other);

    /** \brief Assigment operator. Just increments reference counter */
    ChannelData& operator=(const ChannelData& other);

    /** \brief Allocates internal buffer. */
    void create(size_type sizeBytes);

    /** \brief Allocates internal buffer. */
    void create(size_type rows, size_type colsBytes);

    /** \brief Decrements reference counter and releases internal buffer if nessesary. */
    void release();

    /** \brief Performs swap of pointers to allocated data. */
    void swap(ChannelData& other);

    /** \brief Performs data copying. If destination size differs it will be reallocated */
    void copyTo(ChannelData& other) const;

    /** \brief Returns pointer to y-th row in internal buffer. */
    template<typename T> T* ptr(int y = 0);

    /** \brief Returns pointer to y-th row in internal buffer. */
    template<typename T> const T* ptr(int y = 0) const;

    /** \brief Returns true if unallocated otherwise false. */
    bool empty() const;

    /** \brief Returns allocated rows */
    size_type rows() const;

    /** \brief Returns allocated cols in bytes */
    size_type colsBytes() const;

    /** \brief Returns stride between two consecutive rows in bytes */
    size_t step() const;

    /** \brief Returns reference to density flag */
    bool& dense();

    /** \brief Returns density flag */
    bool dense() const;

  private:
    /** \brief Data pointer */
    void *data_;

    /** \brief Stride between two consecutive rows in bytes */
    size_type step_;

    /** \brief Allocated rows */
    size_type rows_;

    /** \brief Allocated cols in bytes */
    size_type colsBytes_;

    /** \brief Pointer to reference counter */
    int* refcount_;

    /** \brief Density flag */
    bool dense_;
  };

  /////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Channel class
    * \note Templated container class with reference counting.
    * \author Anatoly Baksheev
    */
  template<typename T>
  class PCL_EXPORTS Channel : public ChannelData
  {
  public:

    /** \brief Value type */
    typedef T value_type;

    /** \brief Element size */
    enum { elem_size = sizeof(T) };

    /** \brief Empty constructor */
    Channel();

    /** \brief Allocates internal buffer */
    explicit Channel(size_type size);

    /** \brief Allocates internal buffer */
    Channel(size_type rows, size_type cols);

    /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case. */
    Channel(size_type size, T *data);

    /** \ChannelData Initializes with user allocated buffer. Reference counting is disabled in this case. */
    Channel(size_type rows, size_type cols, T *data);

    /** \ChannelData Initializes with user allocated buffer. Reference counting is disabled in this case. */
    Channel(const std::vector<T>& data);

    /** \brief Copy constructor. Just increments reference counter. */
    Channel(const Channel& other);

    /** \brief Reinterpret constructor. Just increments reference counter. */
    explicit Channel(const ChannelData& other);

    /** \brief Assigment operator. Just increments reference counter. */
    Channel& operator=(const Channel& other);

    /** \brief Allocates internal buffer. */
    void create(size_type size);

    /** \brief Allocates internal buffer. */
    void create(size_type rows, size_type cols);

    /** \brief Decrements reference counter and releases internal buffer if nessesary. */
    void release();

    /** \brief Performs swap of data pointed with another device memory. */
    void swap(Channel& other);

    /** \brief Performs data copying. If destination size differs it will be reallocated. */
    void copyTo(Channel& other) const;

    /** \brief Returns pointer to y-th row in internal buffer. */
    T* ptr(int y = 0);

    /** \brief Returns pointer to y-th row in internal buffer. */
    const T* ptr(int y = 0) const;

    /** \brief Returns allocated rows */
    size_type rows() const;

    /** \brief Returns allocated cols */
    size_type cols() const;
  };

  /////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Cloud class
    * \note Collection of Channels.
    * \author Anatoly Baksheev
    */

  class PCL_EXPORTS Cloud
  {
  public:
    /** \brief Size type */
    typedef size_t size_type;

    /** \brief Empty constructor */
    Cloud();

    /** \brief Adds single Channel */
    template<typename T>
    Cloud(const Channel<T>& Channel);

    /** \brief Acllocates new named Channel. Does nothing if exists with the same size */
    ChannelData create(size_type rows, size_type colsBytes, const std::string& name);

    /** \brief Acllocates nwe buffer by type. Does nothing if exists with the same size */
    template<typename T> Channel<T> create(size_type rows, size_type cols);

    /** \brief Adds typed Channel to the set */
    template<typename T> void set(const Channel<T>& Channel);

    /** \brief Removes typed Channel from the set */
    template<typename T> void remove();

    /** \brief Retrieves typed Channel from the set */
    template<typename T> Channel<T> get();

    /** \brief Retrieves typed Channel from the set */
    template<typename T> const Channel<T> get() const;

    /** \brief Adds named Channel data to the set */
    void set(const ChannelData& data, const std::string& name);

    /** \brief Removes named Channel data from the set */
    void remove(const std::string& name);

    /** \brief Retrieves named Channel data from the set */
    ChannelData get(const std::string& name);

    /** \brief Retrieves named Channel data from the set */
    const ChannelData get(const std::string& name) const;

  private:
    /** \brief Adds Channel data with to the set */
    void set(const ChannelData& Channel_data, int type);

    /** \brief Adds Removes Channel data by type from the set */
    void remove(int type);

    /** \brief Storeage element type */
    struct value_type
    {
      int type_;
      ChannelData data_;
      std::string name_;
    };

    /** \brief Maximal supported set size */
    enum { MAX_ChannelS = 32 };

    /** \brief Channel storage */
    value_type storage_[MAX_ChannelS];

    /** \brief Searches index by name in the storage */
    int name_search(const std::string& name)  const;

    /** \brief Searches index by type in the storage */
    int type_search(int type) const;

    /** \brief Searches empty element in the storage */
    int empty_search()  const;
  };

  template<typename T>
  Cloud& operator+=(Cloud& Channel_set, const Channel<T>& Channel);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Implementation

template<typename T> inline T* pcl::ChannelData::ptr(int y)
{ return (T*)((char*)data_ + colsBytes_ * y); }

template<typename T> inline const T* pcl::ChannelData::ptr(int y) const
{ return (const T*)((const char*)data_ + colsBytes_ * y); }

template<typename T> inline pcl::Channel<T>::Channel() {}
template<typename T> inline pcl::Channel<T>::Channel(size_type size) : ChannelData(size * elem_size) {}
template<typename T> inline pcl::Channel<T>::Channel(size_type rows, size_type cols) : ChannelData(rows, cols * elem_size) {}
template<typename T> inline pcl::Channel<T>::Channel(size_type size, T *data) : ChannelData(size * elem_size, data) {}
template<typename T> inline pcl::Channel<T>::Channel(size_type rows, size_type cols, T *data) : ChannelData(rows, cols * elem_size, data) {}
template<typename T> inline pcl::Channel<T>::Channel(const std::vector<T>& data) : ChannelData(1, data.size() * elem_size, (void*)&data[0]) {}
template<typename T> inline pcl::Channel<T>::Channel(const Channel& other) : ChannelData(other) {}

template<typename T> inline pcl::Channel<T>::Channel(const ChannelData& other)
{ PCL_Assert( other.colsBytes() % sizeof(T) == 0); ChannelData::operator=(other); }

template<typename T> inline pcl::Channel<T>& pcl::Channel<T>::operator=(const Channel& other)
{ ChannelData::operator=(other); return *this; }

template<typename T> inline void pcl::Channel<T>::create(size_type size) { ChannelData::create(size * elem_size); }
template<typename T> inline void pcl::Channel<T>::create(size_type rows, size_type cols) { ChannelData::create(rows, cols * elem_size); }
template<typename T> inline void pcl::Channel<T>::release() { ChannelData::release(); }

template<typename T> inline void pcl::Channel<T>::copyTo(Channel& other) const { ChannelData::copyTo(other); }
template<typename T> inline void pcl::Channel<T>::swap(Channel& other)         { ChannelData::swap(other); }

template<typename T> inline       T* pcl::Channel<T>::ptr(int y)       { return ChannelData::ptr<T>(y); }
template<typename T> inline const T* pcl::Channel<T>::ptr(int y) const { return ChannelData::ptr<T>(y); }

template<typename T> inline pcl::ChannelData::size_type pcl::Channel<T>::rows() const { return ChannelData::rows(); }
template<typename T> inline pcl::ChannelData::size_type pcl::Channel<T>::cols() const { return ChannelData::colsBytes()/elem_size; }

template<typename T> inline pcl::Cloud::Cloud(const Channel<T>& Channel) { set(Channel); }

template<typename T> inline pcl::Channel<T> pcl::Cloud::create(size_type rows, size_type cols)
{
  int i = type_search(channel_traits<T>::type);
  if (i != MAX_ChannelS)
  {
    reinterpret_cast<Channel<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    return Channel<T>(storage_[i].data_);
  }

  i = empty_search();
  if (i != MAX_ChannelS)
  {
    reinterpret_cast<Channel<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    storage_[i].type_ = channel_traits<T>::type;
  }
  else
    PCL_FatalError("Maximal supported Channels limit reached for Cloud");

  return Channel<T>(storage_[i].data_);
}

template<typename T> inline void pcl::Cloud::set(const Channel<T>& Channel) { set(Channel, channel_traits<T>::type); }
template<typename T> inline void pcl::Cloud::remove() { remove(channel_traits<T>::type); }

template<typename T> inline pcl::Channel<T> pcl::Cloud::get()
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_ChannelS ? Channel<T>() : (Channel<T>&)storage_[i].data_;
}

template<typename T> inline const pcl::Channel<T> pcl::Cloud::get() const
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_ChannelS ? Channel<T>() : (Channel<T>&)storage_[i].data_;
}

template<typename T> inline pcl::Cloud& pcl::operator+=(Cloud& Channel_set, const Channel<T>& Channel)
{
  Channel_set.set(Channel); return Channel_set;
}


