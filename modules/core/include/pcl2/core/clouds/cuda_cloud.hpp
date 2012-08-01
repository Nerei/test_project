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

#include <pcl2/core/clouds/cloud.hpp>
#include <pcl2/core/clouds/cuda_kernel_types.hpp>

namespace pcl
{
  namespace cuda
  {
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b CudaCloud class
      * \note This is a BLOB container class with reference counting for GPU memory
      * \author Anatoly Baksheev
      */
    class PCL_EXPORTS CudaData
    {
    public:

      /** \brief Size type */
      typedef size_t size_type;

      /** \brief Empty constructor. */
      CudaData();

      /** \brief Destructor. */
      ~CudaData();

      /** \brief Allocates internal buffer on GPU */
      CudaData(size_type sizeBytes);

      /** \brief Allocates internal buffer on GPU */
      CudaData(size_type rows, size_type colsBytes);

      /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaData(size_type sizeBytes, void *data);

      /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaData(size_type rows, size_type colsBytes, void *data, size_type step);

      /** \brief Copy constructor. Just increments reference counter */
      CudaData(const CudaData& other);

      /** \brief Upload constructor */
      CudaData(const pcl::CloudData& other);

      /** \brief Assigment operator. Just increments reference counter */
      CudaData& operator=(const CudaData& other);

       /** \brief Allocates internal buffer on GPU */
      void create(size_t sizeBytes);

      /** \brief Allocates internal buffer. */
      void create(size_type rows, size_type colsBytes);

      /** \brief Decrements reference counter and releases internal buffer if nessesary */
      void release();

      /** \brief Performs swap of pointers to allocated data. */
      void swap(CudaData& other);

      /** \brief Performs data copying. If destination size differs it will be reallocated */
      void copyTo(CudaData& other) const;

      /** \brief Returns pointer to y-th row for internal buffer in GPU memory. */
      template<typename T> T* ptr(int y = 0);

      /** \brief Returns pointer to y-th row for internal buffer in GPU memory. */
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

      /** \brief Returns true if internal buffer is allocated as pitched */
      bool pitched() const;

      /** \brief Uploads data to internal buffer in GPU memory */
      void upload(const CloudData& cloud_data);

      /** \brief Downloads data from internal buffer to CPU memory */
      void download(CloudData& cloud_data) const;

      /** \brief Conversion to PtrSz for passing to kernel functions. */
      template <class U> operator PtrSz<U>() const;

      /** \brief Conversion to PtrStep for passing to kernel functions. */
      template <class U> operator PtrStep<U>() const;

      /** \brief Conversion to PtrStepSz for passing to kernel functions. */
      template <class U> operator PtrStepSz<U>() const;

    private:
      /** \brief Data pointer to GPU memory */
      void *data_;

      /** \brief Stride between two consecutive rows in bytes */
      size_type step_;

      /** \brief Allocated rows */
      size_type rows_;

      /** \brief Allocated cols in bytes */
      size_type colsBytes_;

      /** \brief Pointer to reference counter in CPU memory */
      int* refcount_;

      /** \brief Density flag */
      bool dense_;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Cloud class
      * \note Templated container class with reference counting for GPU memory
      * \author Anatoly Baksheev
      */
    template<typename T>
    class PCL_EXPORTS CudaCloud : public CudaData
    {
    public:
      /** \brief Value type */
      typedef T value_type;

      /** \brief Element size */
      enum { elem_size = sizeof(T) };

      /** \brief Empty constructor. */
      CudaCloud();

      /** \brief Allocates internal buffer */
      explicit CudaCloud(size_type size);

      /** \brief Allocates internal buffer */
      CudaCloud(size_type rows, size_type cols);

      /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaCloud(size_type size, T *data);

      /** \CloudData Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaCloud(size_type rows, size_type cols, T *data, size_type step);

      /** \brief Copy constructor. Just increments reference counter */
      CudaCloud(const CudaCloud& other);

      /** \brief Reinterpret constructor. Just increments reference counter */
      explicit CudaCloud(const CudaData& other);

      /** \brief Upload constructor */
      explicit CudaCloud(const Cloud<T>& other);

      /** \brief Assigment operator. Just increments reference counter */
      CudaCloud& operator=(const CudaCloud& other);

      /** \brief Allocates internal buffer. */
      void create(size_type size);

      /** \brief Allocates internal buffer. */
      void create(size_type rows, size_type cols);

      /** \brief Decrements reference counter and releases internal buffer if nessesary. */
      void release();

      /** \brief Performs swap of data pointed with another device memory. */
      void swap(CudaCloud& other);

      /** \brief Performs data copying. If destination size differs it will be reallocated. */
      void copyTo(CudaCloud& other) const;

      /** \brief Returns pointer to y-th row in internal buffer. */
      T* ptr(int y = 0);

      /** \brief Returns pointer to y-th row in internal buffer. */
      const T* ptr(int y = 0) const;

      /** \brief Returns allocated rows */
      size_type rows() const;

      /** \brief Returns allocated cols */
      size_type cols() const;

      /** \brief Uploads data to internal buffer in GPU memory */
      void upload(const Cloud<T>& cloud);

      /** \brief Downloads data from internal buffer to CPU memory */
      void download(Cloud<T>& cloud) const;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b CloudSet class
      * \note Collection of clouds.
      * \author Anatoly Baksheev
      */

    class CudaCloudSet
    {
    public:
      /** \brief Size type */
      typedef size_t size_type;

      /** \brief Empty constructor */
      CudaCloudSet();

      /** \brief Adds single cloud */
      template<typename T>
      CudaCloudSet(const CudaCloud<T>& cloud);

      /** \brief Acllocates new named cloud. Does nothing if exists with the same size */
      CudaData create(size_type rows, size_type colsBytes, const std::string& name);

      /** \brief Acllocates nwe buffer by type. Does nothing if exists with the same size */
      template<typename T> CudaCloud<T> create(size_type rows, size_type cols);

      /** \brief Adds typed cloud to the set */
      template<typename T> void set(const CudaCloud<T>& cloud);

      /** \brief Removes typed cloud from the set */
      template<typename T> void remove();

      /** \brief Retrieves typed cloud from the set */
      template<typename T> CudaCloud<T> get();

      /** \brief Retrieves typed cloud from the set */
      template<typename T> const CudaCloud<T> get() const;

      /** \brief Adds named cloud data to the set */
      void set(const CudaData& data, const std::string& name);

      /** \brief Removes named cloud data from the set */
      void remove(const std::string& name);

      /** \brief Retrieves named cloud data from the set */
      CudaData get(const std::string& name);

      /** \brief Retrieves named cloud data from the set */
      const CudaData get(const std::string& name) const;

    private:
      /** \brief Adds cloud data with to the set */
      void set(const CudaData& cloud_data, int type);

      /** \brief Adds Removes cloud data by type from the set */
      void remove(int type);

      /** \brief Storeage element type */
      struct value_type
      {
        int type_;
        CudaData data_;
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
    CudaCloudSet& operator+=(CudaCloudSet& cloud_set, const CudaCloud<T>& cloud);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Implementation

template<typename T> inline T* pcl::cuda::CudaData::ptr(int y)
{ return (T*)((char*)data_ + colsBytes_ * y); }

template<typename T> inline const T* pcl::cuda::CudaData::ptr(int y) const
{ return (const T*)((const char*)data_ + colsBytes_ * y); }

template <class U> inline pcl::cuda::CudaData::operator pcl::cuda::PtrSz<U>() const
{
  PtrSz<U> result;
  result.data = (U*)ptr<U>();
  result.size = colsBytes_/sizeof(U);
  return result;
}

template <class U> inline pcl::cuda::CudaData::operator pcl::cuda::PtrStep<U>() const
{
  PtrStepSz<U> result;
  result.data = (U*)ptr<U>();
  result.step = step_;
  return result;
}

template <class U> inline pcl::cuda::CudaData::operator pcl::cuda::PtrStepSz<U>() const
{
  PtrStepSz<U> result;
  result.data = (U*)ptr<U>();
  result.step = step_;
  result.cols = colsBytes_/sizeof(U);
  result.rows = rows_;
  return result;
}

template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud() {}
template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(size_type size) : CudaData(size * elem_size) {}
template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(size_type rows, size_type cols) : CudaData(rows, cols * elem_size) {}
template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(size_type size, T *data) : CudaData(size * elem_size, data) {}
template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(size_type rows, size_type cols, T *data, size_type step) : CudaData(rows, cols * elem_size, data, step) {}
template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(const CudaCloud& other) : CudaData(other) {}
template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(const Cloud<T>& other) { upload(other); }

template<typename T> inline pcl::cuda::CudaCloud<T>::CudaCloud(const CudaData& other)
{ CudaData::operator=(other); }

template<typename T> inline pcl::cuda::CudaCloud<T>& pcl::cuda::CudaCloud<T>::operator=(const CudaCloud& other)
{ CudaData::operator=(other); return *this; }

template<typename T> inline void pcl::cuda::CudaCloud<T>::create(size_type size) { CudaData::create(size * elem_size); }
template<typename T> inline void pcl::cuda::CudaCloud<T>::create(size_type rows, size_type cols) { CudaData::create(rows, cols * elem_size); }
template<typename T> inline void pcl::cuda::CudaCloud<T>::release() { CudaData::release(); }

template<typename T> inline void pcl::cuda::CudaCloud<T>::copyTo(CudaCloud& other) const { CudaData::copyTo(other); }
template<typename T> inline void pcl::cuda::CudaCloud<T>::swap(CudaCloud& other)         { CudaData::swap(other); }

template<typename T> inline       T* pcl::cuda::CudaCloud<T>::ptr(int y)       { return CudaData::ptr<T>(y); }
template<typename T> inline const T* pcl::cuda::CudaCloud<T>::ptr(int y) const { return CudaData::ptr<T>(y); }

template<typename T> inline typename pcl::cuda::CudaCloud<T>::size_type pcl::cuda::CudaCloud<T>::rows() const { return CudaData::rows(); }
template<typename T> inline typename pcl::cuda::CudaCloud<T>::size_type pcl::cuda::CudaCloud<T>::cols() const { return CudaData::colsBytes()/elem_size; }

template<typename T> inline void pcl::cuda::CudaCloud<T>::upload(const Cloud<T>& cloud) { CudaData::upload(cloud); }
template<typename T> inline void pcl::cuda::CudaCloud<T>::download(Cloud<T>& cloud) const { CudaData::download(cloud); }


template<typename T> inline pcl::cuda::CudaCloudSet::CudaCloudSet(const CudaCloud<T>& cloud) { set(cloud); }

template<typename T> inline pcl::cuda::CudaCloud<T> pcl::cuda::CudaCloudSet::create(size_type rows, size_type cols)
{
  int i = type_search(channel_traits<T>::type);
  if (i != MAX_CLOUDS)
  {
    reinterpret_cast<CudaCloud<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    return storage_[i].data_;
  }

  i = empty_search();
  if (i != MAX_CLOUDS)
  {
    reinterpret_cast<CudaCloud<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    storage_[i].type_ = channel_traits<T>::type;
  }
  else
    PCL_FatalError("Maximal supported clouds limit reached for CloudSet");

  return storage_[i].data_;
}


template<typename T> void pcl::cuda::CudaCloudSet::set(const CudaCloud<T>& cloud) { set(cloud, channel_traits<T>::type); }
template<typename T> void pcl::cuda::CudaCloudSet::remove() { remove(channel_traits<T>::type); }

template<typename T> pcl::cuda::CudaCloud<T> pcl::cuda::CudaCloudSet::get()
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_CLOUDS ? CudaCloud<T>() : (CudaCloud<T>&)storage_[i].data_;
}

template<typename T> const pcl::cuda::CudaCloud<T> pcl::cuda::CudaCloudSet::get() const
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_CLOUDS ? CudaCloud<T>() : (CudaCloud<T>&)storage_[i].data_;
}


template<typename T> inline pcl::cuda::CudaCloudSet& pcl::cuda::operator+=(CudaCloudSet& cloud_set, const CudaCloud<T>& cloud)
{
  cloud_set.set(cloud); return cloud_set;
}
