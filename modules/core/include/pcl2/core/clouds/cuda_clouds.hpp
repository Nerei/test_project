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

#include <pcl2/core/clouds/clouds.hpp>
#include <pcl2/core/clouds/cuda_kernel_types.hpp>

namespace pcl
{
  namespace cuda
  {
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b CudaChannel class
      * \note This is a BLOB container class with reference counting for GPU memory
      * \author Anatoly Baksheev
      */
    class PCL_EXPORTS CudaChannelData
    {
    public:

      /** \brief Size type */
      typedef size_t size_type;

      /** \brief Empty constructor. */
      CudaChannelData();

      /** \brief Destructor. */
      ~CudaChannelData();

      /** \brief Allocates internal buffer on GPU */
      CudaChannelData(size_type sizeBytes);

      /** \brief Allocates internal buffer on GPU */
      CudaChannelData(size_type rows, size_type colsBytes);

      /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaChannelData(size_type sizeBytes, void *data);

      /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaChannelData(size_type rows, size_type colsBytes, void *data, size_type step);

      /** \brief Copy constructor. Just increments reference counter */
      CudaChannelData(const CudaChannelData& other);

      /** \brief Upload constructor */
      CudaChannelData(const pcl::ChannelData& other);

      /** \brief Assigment operator. Just increments reference counter */
      CudaChannelData& operator=(const CudaChannelData& other);

       /** \brief Allocates internal buffer on GPU */
      void create(size_t sizeBytes);

      /** \brief Allocates internal buffer. */
      void create(size_type rows, size_type colsBytes);

      /** \brief Decrements reference counter and releases internal buffer if nessesary */
      void release();

      /** \brief Performs swap of pointers to allocated data. */
      void swap(CudaChannelData& other);

      /** \brief Performs data copying. If destination size differs it will be reallocated */
      void copyTo(CudaChannelData& other) const;

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
      void upload(const ChannelData& Channel_data);

      /** \brief Downloads data from internal buffer to CPU memory */
      void download(ChannelData& Channel_data) const;

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
    /** \brief @b Channel class
      * \note Templated container class with reference counting for GPU memory
      * \author Anatoly Baksheev
      */
    template<typename T>
    class PCL_EXPORTS CudaChannel : public CudaChannelData
    {
    public:
      /** \brief Value type */
      typedef T value_type;

      /** \brief Element size */
      enum { elem_size = sizeof(T) };

      /** \brief Empty constructor. */
      CudaChannel();

      /** \brief Allocates internal buffer */
      explicit CudaChannel(size_type size);

      /** \brief Allocates internal buffer */
      CudaChannel(size_type rows, size_type cols);

      /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaChannel(size_type size, T *data);

      /** \ChannelData Initializes with user allocated buffer. Reference counting is disabled in this case */
      CudaChannel(size_type rows, size_type cols, T *data, size_type step);

      /** \brief Copy constructor. Just increments reference counter */
      CudaChannel(const CudaChannel& other);

      /** \brief Reinterpret constructor. Just increments reference counter */
      explicit CudaChannel(const CudaChannelData& other);

      /** \brief Upload constructor */
      explicit CudaChannel(const Channel<T>& other);

      /** \brief Assigment operator. Just increments reference counter */
      CudaChannel& operator=(const CudaChannel& other);

      /** \brief Allocates internal buffer. */
      void create(size_type size);

      /** \brief Allocates internal buffer. */
      void create(size_type rows, size_type cols);

      /** \brief Decrements reference counter and releases internal buffer if nessesary. */
      void release();

      /** \brief Performs swap of data pointed with another device memory. */
      void swap(CudaChannel& other);

      /** \brief Performs data copying. If destination size differs it will be reallocated. */
      void copyTo(CudaChannel& other) const;

      /** \brief Returns pointer to y-th row in internal buffer. */
      T* ptr(int y = 0);

      /** \brief Returns pointer to y-th row in internal buffer. */
      const T* ptr(int y = 0) const;

      /** \brief Returns allocated rows */
      size_type rows() const;

      /** \brief Returns allocated cols */
      size_type cols() const;

      /** \brief Uploads data to internal buffer in GPU memory */
      void upload(const Channel<T>& Channel);

      /** \brief Downloads data from internal buffer to CPU memory */
      void download(Channel<T>& Channel) const;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Cloud class
      * \note Collection of Channels.
      * \author Anatoly Baksheev
      */

    class CudaCloud
    {
    public:
      /** \brief Size type */
      typedef size_t size_type;

      /** \brief Empty constructor */
      CudaCloud();

      /** \brief Adds single Channel */
      template<typename T>
      CudaCloud(const CudaChannel<T>& Channel);

      /** \brief Acllocates new named Channel. Does nothing if exists with the same size */
      CudaChannelData create(size_type rows, size_type colsBytes, const std::string& name);

      /** \brief Acllocates nwe buffer by type. Does nothing if exists with the same size */
      template<typename T> CudaChannel<T> create(size_type rows, size_type cols);

      /** \brief Adds typed Channel to the set */
      template<typename T> void set(const CudaChannel<T>& Channel);

      /** \brief Removes typed Channel from the set */
      template<typename T> void remove();

      /** \brief Retrieves typed Channel from the set */
      template<typename T> CudaChannel<T> get();

      /** \brief Retrieves typed Channel from the set */
      template<typename T> const CudaChannel<T> get() const;

      /** \brief Adds named Channel data to the set */
      void set(const CudaChannelData& data, const std::string& name);

      /** \brief Removes named Channel data from the set */
      void remove(const std::string& name);

      /** \brief Retrieves named Channel data from the set */
      CudaChannelData get(const std::string& name);

      /** \brief Retrieves named Channel data from the set */
      const CudaChannelData get(const std::string& name) const;

    private:
      /** \brief Adds Channel data with to the set */
      void set(const CudaChannelData& Channel_data, int type);

      /** \brief Adds Removes Channel data by type from the set */
      void remove(int type);

      /** \brief Storeage element type */
      struct value_type
      {
        int type_;
        CudaChannelData data_;
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
    CudaCloud& operator+=(CudaCloud& Channel_set, const CudaChannel<T>& Channel);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Implementation

template<typename T> inline T* pcl::cuda::CudaChannelData::ptr(int y)
{ return (T*)((char*)data_ + colsBytes_ * y); }

template<typename T> inline const T* pcl::cuda::CudaChannelData::ptr(int y) const
{ return (const T*)((const char*)data_ + colsBytes_ * y); }

template <class U> inline pcl::cuda::CudaChannelData::operator pcl::cuda::PtrSz<U>() const
{
  PtrSz<U> result;
  result.data = (U*)ptr<U>();
  result.size = colsBytes_/sizeof(U);
  return result;
}

template <class U> inline pcl::cuda::CudaChannelData::operator pcl::cuda::PtrStep<U>() const
{
  PtrStepSz<U> result;
  result.data = (U*)ptr<U>();
  result.step = step_;
  return result;
}

template <class U> inline pcl::cuda::CudaChannelData::operator pcl::cuda::PtrStepSz<U>() const
{
  PtrStepSz<U> result;
  result.data = (U*)ptr<U>();
  result.step = step_;
  result.cols = colsBytes_/sizeof(U);
  result.rows = rows_;
  return result;
}

template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel() {}
template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(size_type size) : CudaChannelData(size * elem_size) {}
template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(size_type rows, size_type cols) : CudaChannelData(rows, cols * elem_size) {}
template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(size_type size, T *data) : CudaChannelData(size * elem_size, data) {}
template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(size_type rows, size_type cols, T *data, size_type step) : CudaChannelData(rows, cols * elem_size, data, step) {}
template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(const CudaChannel& other) : CudaChannelData(other) {}
template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(const Channel<T>& other) { upload(other); }

template<typename T> inline pcl::cuda::CudaChannel<T>::CudaChannel(const CudaChannelData& other)
{ CudaChannelData::operator=(other); }

template<typename T> inline pcl::cuda::CudaChannel<T>& pcl::cuda::CudaChannel<T>::operator=(const CudaChannel& other)
{ CudaChannelData::operator=(other); return *this; }

template<typename T> inline void pcl::cuda::CudaChannel<T>::create(size_type size) { CudaChannelData::create(size * elem_size); }
template<typename T> inline void pcl::cuda::CudaChannel<T>::create(size_type rows, size_type cols) { CudaChannelData::create(rows, cols * elem_size); }
template<typename T> inline void pcl::cuda::CudaChannel<T>::release() { CudaChannelData::release(); }

template<typename T> inline void pcl::cuda::CudaChannel<T>::copyTo(CudaChannel& other) const { CudaChannelData::copyTo(other); }
template<typename T> inline void pcl::cuda::CudaChannel<T>::swap(CudaChannel& other)         { CudaChannelData::swap(other); }

template<typename T> inline       T* pcl::cuda::CudaChannel<T>::ptr(int y)       { return CudaChannelData::ptr<T>(y); }
template<typename T> inline const T* pcl::cuda::CudaChannel<T>::ptr(int y) const { return CudaChannelData::ptr<T>(y); }

template<typename T> inline typename pcl::cuda::CudaChannel<T>::size_type pcl::cuda::CudaChannel<T>::rows() const { return CudaChannelData::rows(); }
template<typename T> inline typename pcl::cuda::CudaChannel<T>::size_type pcl::cuda::CudaChannel<T>::cols() const { return CudaChannelData::colsBytes()/elem_size; }

template<typename T> inline void pcl::cuda::CudaChannel<T>::upload(const Channel<T>& Channel) { CudaChannelData::upload(Channel); }
template<typename T> inline void pcl::cuda::CudaChannel<T>::download(Channel<T>& Channel) const { CudaChannelData::download(Channel); }


template<typename T> inline pcl::cuda::CudaCloud::CudaCloud(const CudaChannel<T>& Channel) { set(Channel); }

template<typename T> inline pcl::cuda::CudaChannel<T> pcl::cuda::CudaCloud::create(size_type rows, size_type cols)
{
  int i = type_search(channel_traits<T>::type);
  if (i != MAX_ChannelS)
  {
    reinterpret_cast<CudaChannel<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    return storage_[i].data_;
  }

  i = empty_search();
  if (i != MAX_ChannelS)
  {
    reinterpret_cast<CudaChannel<T>&>(storage_[i].data_).create(rows, cols);
    storage_[i].name_.clear();
    storage_[i].type_ = channel_traits<T>::type;
  }
  else
    PCL_FatalError("Maximal supported Channels limit reached for Cloud");

  return storage_[i].data_;
}


template<typename T> void pcl::cuda::CudaCloud::set(const CudaChannel<T>& Channel) { set(Channel, channel_traits<T>::type); }
template<typename T> void pcl::cuda::CudaCloud::remove() { remove(channel_traits<T>::type); }

template<typename T> pcl::cuda::CudaChannel<T> pcl::cuda::CudaCloud::get()
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_ChannelS ? CudaChannel<T>() : (CudaChannel<T>&)storage_[i].data_;
}

template<typename T> const pcl::cuda::CudaChannel<T> pcl::cuda::CudaCloud::get() const
{
  int i = type_search(channel_traits<T>::type);
  return i == MAX_ChannelS ? CudaChannel<T>() : (CudaChannel<T>&)storage_[i].data_;
}


template<typename T> inline pcl::cuda::CudaCloud& pcl::cuda::operator+=(CudaCloud& Channel_set, const CudaChannel<T>& Channel)
{
  Channel_set.set(Channel); return Channel_set;
}
