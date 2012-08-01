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
  /** \brief @b CloudData class
    * \note BLOB container class with reference counting.
    * \author Anatoly Baksheev
    */
  class PCL_EXPORTS CloudData
  {
  public:

    /** \brief Size type */
    typedef size_t size_type;

    /** \brief Empty constructor */
    CloudData();

    /** \brief Destructor */
    ~CloudData();

    /** \brief Allocates internal buffer */
    CloudData(size_type sizeBytes);

    /** \brief Allocates internal buffer */
    CloudData(size_type rows, size_type colsBytes);

    /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
    CloudData(size_type sizeBytes, void *data);

    /** \CloudData Initializes with user allocated buffer. Reference counting is disabled in this case */
    CloudData(size_type rows, size_type colsBytes, void *data);

    /** \brief Copy constructor. Just increments reference counter */
    CloudData(const CloudData& other);

    /** \brief Assigment operator. Just increments reference counter */
    CloudData& operator=(const CloudData& other);

    /** \brief Allocates internal buffer. */
    void create(size_type sizeBytes);

    /** \brief Allocates internal buffer. */
    void create(size_type rows, size_type colsBytes);

    /** \brief Decrements reference counter and releases internal buffer if nessesary. */
    void release();

    /** \brief Performs swap of pointers to allocated data. */
    void swap(CloudData& other);

    /** \brief Performs data copying. If destination size differs it will be reallocated */
    void copyTo(CloudData& other) const;

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
  /** \brief @b Cloud class
    * \note Templated container class with reference counting.
    * \author Anatoly Baksheev
    */
  template<typename T>
  class PCL_EXPORTS Cloud : public CloudData
  {
  public:

    /** \brief Value type */
    typedef T value_type;

    /** \brief Element size */
    enum { elem_size = sizeof(T) };

    /** \brief Empty constructor */
    Cloud();

    /** \brief Allocates internal buffer */
    explicit Cloud(size_type size);

    /** \brief Allocates internal buffer */
    Cloud(size_type rows, size_type cols);

    /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case. */
    Cloud(size_type size, T *data);

    /** \CloudData Initializes with user allocated buffer. Reference counting is disabled in this case. */
    Cloud(size_type rows, size_type cols, T *data);

    /** \CloudData Initializes with user allocated buffer. Reference counting is disabled in this case. */
    Cloud(const std::vector<T>& data);

    /** \brief Copy constructor. Just increments reference counter. */
    Cloud(const Cloud& other);

    /** \brief Reinterpret constructor. Just increments reference counter. */
    explicit Cloud(const CloudData& other);

    /** \brief Assigment operator. Just increments reference counter. */
    Cloud& operator=(const Cloud& other);

    /** \brief Allocates internal buffer. */
    void create(size_type size);

    /** \brief Allocates internal buffer. */
    void create(size_type rows, size_type cols);

    /** \brief Decrements reference counter and releases internal buffer if nessesary. */
    void release();

    /** \brief Performs swap of data pointed with another device memory. */
    void swap(Cloud& other);

    /** \brief Performs data copying. If destination size differs it will be reallocated. */
    void copyTo(Cloud& other) const;

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
  /** \brief @b CloudSet class
    * \note Collection of clouds.
    * \author Anatoly Baksheev
    */

  class PCL_EXPORTS CloudSet
  {
  public:
    /** \brief Size type */
    typedef size_t size_type;

    /** \brief Empty constructor */
    CloudSet();

    /** \brief Adds single cloud */
    template<typename T>
    CloudSet(const Cloud<T>& cloud);

    /** \brief Acllocates new named cloud. Does nothing if exists with the same size */
    CloudData create(size_type rows, size_type colsBytes, const std::string& name);

    /** \brief Acllocates nwe buffer by type. Does nothing if exists with the same size */
    template<typename T> Cloud<T> create(size_type rows, size_type cols);

    /** \brief Adds typed cloud to the set */
    template<typename T> void set(const Cloud<T>& cloud);

    /** \brief Removes typed cloud from the set */
    template<typename T> void remove();

    /** \brief Retrieves typed cloud from the set */
    template<typename T> Cloud<T> get();

    /** \brief Retrieves typed cloud from the set */
    template<typename T> const Cloud<T> get() const;

    /** \brief Adds named cloud data to the set */
    void set(const CloudData& data, const std::string& name);

    /** \brief Removes named cloud data from the set */
    void remove(const std::string& name);

    /** \brief Retrieves named cloud data from the set */
    CloudData get(const std::string& name);

    /** \brief Retrieves named cloud data from the set */
    const CloudData get(const std::string& name) const;

  private:
    /** \brief Adds cloud data with to the set */
    void set(const CloudData& cloud_data, int type);

    /** \brief Adds Removes cloud data by type from the set */
    void remove(int type);

    /** \brief Storeage element type */
    struct value_type
    {
      int type_;
      CloudData data_;
      std::string name_;
    };

    /** \brief Maximal supported set size */
    enum { MAX_CLOUDS = 32 };

    /** \brief Cloud storage */
    value_type storage_[MAX_CLOUDS];

    /** \brief Searches index by name in the storage */
    int name_search(const std::string& name)  const;

    /** \brief Searches index by type in the storage */
    int type_search(int type) const;

    /** \brief Searches empty element in the storage */
    int empty_search()  const;
  };

  template<typename T>
  CloudSet& operator+=(CloudSet& cloud_set, const Cloud<T>& cloud);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Implementation

template<typename T> inline T* pcl::CloudData::ptr(int y)
{ return (T*)((char*)data_ + colsBytes_ * y); }

template<typename T> inline const T* pcl::CloudData::ptr(int y) const
{ return (const T*)((const char*)data_ + colsBytes_ * y); }

template<typename T> inline pcl::Cloud<T>::Cloud() {}
template<typename T> inline pcl::Cloud<T>::Cloud(size_type size) : CloudData(size * elem_size) {}
template<typename T> inline pcl::Cloud<T>::Cloud(size_type rows, size_type cols) : CloudData(rows, cols * elem_size) {}
template<typename T> inline pcl::Cloud<T>::Cloud(size_type size, T *data) : CloudData(size * elem_size, data) {}
template<typename T> inline pcl::Cloud<T>::Cloud(size_type rows, size_type cols, T *data) : CloudData(rows, cols * elem_size, data) {}
template<typename T> inline pcl::Cloud<T>::Cloud(const std::vector<T>& data) : CloudData(1, data.size() * elem_size, (void*)&data[0]) {}
template<typename T> inline pcl::Cloud<T>::Cloud(const Cloud& other) : CloudData(other) {}

template<typename T> inline pcl::Cloud<T>::Cloud(const CloudData& other)
{ PCL_Assert( other.colsBytes() % sizeof(T) == 0); CloudData::operator=(other); }

template<typename T> inline pcl::Cloud<T>& pcl::Cloud<T>::operator=(const Cloud& other)
{ CloudData::operator=(other); return *this; }

template<typename T> inline void pcl::Cloud<T>::create(size_type size) { CloudData::create(size * elem_size); }
template<typename T> inline void pcl::Cloud<T>::create(size_type rows, size_type cols) { CloudData::create(rows, cols * elem_size); }
template<typename T> inline void pcl::Cloud<T>::release() { CloudData::release(); }

template<typename T> inline void pcl::Cloud<T>::copyTo(Cloud& other) const { CloudData::copyTo(other); }
template<typename T> inline void pcl::Cloud<T>::swap(Cloud& other)         { CloudData::swap(other); }

template<typename T> inline       T* pcl::Cloud<T>::ptr(int y)       { return CloudData::ptr<T>(y); }
template<typename T> inline const T* pcl::Cloud<T>::ptr(int y) const { return CloudData::ptr<T>(y); }

template<typename T> inline pcl::CloudData::size_type pcl::Cloud<T>::rows() const { return CloudData::rows(); }
template<typename T> inline pcl::CloudData::size_type pcl::Cloud<T>::cols() const { return CloudData::colsBytes()/elem_size; }

template<typename T> inline pcl::CloudSet::CloudSet(const Cloud<T>& cloud) { set(cloud); }

template<typename T> inline pcl::Cloud<T> pcl::CloudSet::create(size_type rows, size_type cols)
{
  int i = type_search(channel_traits<T>::type);
  if (i != MAX_CLOUDS)
  {
    reinterpret_cast<Cloud<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    return Cloud<T>(storage_[i].data_);
  }

  i = empty_search();
  if (i != MAX_CLOUDS)
  {
    reinterpret_cast<Cloud<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    storage_[i].type_ = channel_traits<T>::type;
  }
  else
    PCL_FatalError("Maximal supported clouds limit reached for CloudSet");

  return Cloud<T>(storage_[i].data_);
}

template<typename T> inline void pcl::CloudSet::set(const Cloud<T>& cloud) { set(cloud, channel_traits<T>::type); }
template<typename T> inline void pcl::CloudSet::remove() { remove(channel_traits<T>::type); }

template<typename T> inline pcl::Cloud<T> pcl::CloudSet::get()
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_CLOUDS ? Cloud<T>() : (Cloud<T>&)storage_[i].data_;
}

template<typename T> inline const pcl::Cloud<T> pcl::CloudSet::get() const
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_CLOUDS ? Cloud<T>() : (Cloud<T>&)storage_[i].data_;
}

template<typename T> inline pcl::CloudSet& pcl::operator+=(CloudSet& cloud_set, const Cloud<T>& cloud)
{
  cloud_set.set(cloud); return cloud_set;
}


