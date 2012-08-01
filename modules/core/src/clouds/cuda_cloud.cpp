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

using namespace pcl;


/////////////////////////////////////////////////////////////////////////////////////////////////
// CudaData

#if !defined HAVE_CUDA

void throw_nocuda() { PCL_FatalError("The library was compiled without CUDA support"); }

pcl::cuda::CudaData::CudaData() { throw_nocuda(); }       
pcl::cuda::CudaData::~CudaData() { throw_nocuda(); }       
pcl::cuda::CudaData::CudaData(size_type /*sizeBytes*/) { throw_nocuda(); }
pcl::cuda::CudaData::CudaData(size_type /*rows*/, size_type /*colsBytes*/) { throw_nocuda(); }
pcl::cuda::CudaData::CudaData(size_type /*sizeBytes*/, void* /*data*/) { throw_nocuda(); }
pcl::cuda::CudaData::CudaData(const CudaData& /*other*/) { throw_nocuda(); }
pcl::cuda::CudaData& pcl::cuda::CudaData::operator=(const CudaData& /*other*/) { throw_nocuda(); return *this; }
void pcl::cuda::CudaData::create(size_t /*sizeBytes*/) { throw_nocuda(); }
void pcl::cuda::CudaData::release() { throw_nocuda(); }
void pcl::cuda::CudaData::copyTo(CudaData& /*other*/) const { throw_nocuda(); }
void pcl::cuda::CudaData::swap(CudaData& other) { throw_nocuda(); }
bool pcl::cuda::CudaData::empty() const { throw_nocuda(); return true; }          
pcl::CloudData::size_type pcl::cuda::CudaData::colsBytes() const { throw_nocuda(); return 0; }

void pcl::cuda::CudaData::upload(const CloudData& /*cloud_data*/) { throw_nocuda(); }
void pcl::cuda::CudaData::download(CloudData& /*cloud_data*/) const { throw_nocuda(); }

#else

pcl::cuda::CudaData::CudaData() : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) {} 
pcl::cuda::CudaData::CudaData(size_type sizeBytes) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) 
{ create(sizeBytes); } 

pcl::cuda::CudaData::CudaData(size_type rows, size_type colsBytes) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true)
{ create(rows, colsBytes); } 

pcl::cuda::CudaData::CudaData(size_type sizeBytes, void *data) : data_((unsigned char*)data), step_(0), rows_(1), colsBytes_(sizeBytes), refcount_(0), dense_(true) {}

pcl::cuda::CudaData::CudaData(size_type rows, size_type colsBytes, void *data, size_type step) 
   : data_((unsigned char*)data), step_(step), rows_(rows), colsBytes_(colsBytes), refcount_(0), dense_(true) {}
          
pcl::cuda::CudaData::~CudaData() { release(); }


pcl::cuda::CudaData::CudaData(const CudaData& other) : data_(other.data_), step_(other.step_), rows_(other.rows_), 
    colsBytes_(other.colsBytes_), refcount_(other.refcount_), dense_(other.dense_)
{
  if( refcount_ )
    __XADD__(refcount_, 1);
}

pcl::cuda::CudaData::CudaData(const CloudData& other) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) 
{ upload(other); } 

pcl::cuda::CudaData& pcl::cuda::CudaData::operator=(const CudaData& other)
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
 
void pcl::cuda::CudaData::create(size_t sizeBytes) 
{  
  if (!pitched() && rows_ == 1 && colsBytes_ == sizeBytes)
    return;
            
  if(sizeBytes > 0)
  {        
    if( data_ )
      release();

    colsBytes_ = sizeBytes;
    rows_ = 1;
    step_ = 0;

    cudaSafeCall( cudaMalloc( (void**)&data_, colsBytes_) );        

    refcount_ = new int;
    *refcount_ = 1;
  }
}

void pcl::cuda::CudaData::create(size_type rows, size_type colsBytes)
{
  if (rows  == rows_ && colsBytes == colsBytes_ && pitched())
    return;
            
  if(rows * colsBytes > 0)
  {        
    if( data_ )
      release();
    
    colsBytes_ = colsBytes;    
    rows_ = rows;
    
    cudaSafeCall( cudaMallocPitch( (void**)&data_, &step_, colsBytes_, rows_) );        

    refcount_ = new int;
    *refcount_ = 1;
  }
}
          
void pcl::cuda::CudaData::release()
{
  if( refcount_ && __XADD__(refcount_, -1) == 1 )
  {  
    delete refcount_;
    cudaSafeCall( cudaFree(data_) );
  }

  colsBytes_ = 0;
  rows_ = 0;    
  data_ = 0;    
  step_ = 0;
  refcount_ = 0;
}

void pcl::cuda::CudaData::swap(CudaData& other)
{
  std::swap(data_, other.data_);
  std::swap(step_, other.step_);
  std::swap(rows_, other.rows_);
  std::swap(colsBytes_, other.colsBytes_);
  std::swap(refcount_, other.refcount_);
  std::swap(dense_, other.dense_);
}

void pcl::cuda::CudaData::copyTo(CudaData& other) const
{
  if (empty())  
  {
    other.release();
    return;
  }
  
  if (!pitched())
  {
    other.create(colsBytes_);
    cudaSafeCall( cudaMemcpy(other.data_, data_, colsBytes_, cudaMemcpyDeviceToDevice) );    
  }
  else
  {
    other.create(rows_, colsBytes_);
    cudaSafeCall( cudaMemcpy2D(other.data_, other.step_, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToDevice) );  
  }
  cudaSafeCall( cudaDeviceSynchronize() );
}

bool pcl::cuda::CudaData::empty() const { return !data_; }
pcl::cuda::CudaData::size_type pcl::cuda::CudaData::rows() const { return rows_; }
pcl::cuda::CudaData::size_type pcl::cuda::CudaData::colsBytes() const { return colsBytes_; }
pcl::cuda::CudaData::size_type pcl::cuda::CudaData::step() const { return step_; }
bool& pcl::cuda::CudaData::dense() { return dense_; }
bool  pcl::cuda::CudaData::dense() const  { return dense_; }  
bool  pcl::cuda::CudaData::pitched() const { return step() != 0; }

void pcl::cuda::CudaData::upload(const CloudData& cloud_data)
{
  if (cloud_data.rows() == 1)
  {
    create(cloud_data.colsBytes());
    cudaSafeCall( cudaMemcpy(data_, cloud_data.ptr<void>(), colsBytes_, cudaMemcpyHostToDevice) );      
  }
  else
  {
    create(cloud_data.rows(), cloud_data.colsBytes());
    cudaSafeCall( cudaMemcpy2D(data_, step_, cloud_data.ptr<void>(), cloud_data.step(), colsBytes_, rows_, cudaMemcpyHostToDevice) );  
  }
  cudaSafeCall( cudaDeviceSynchronize() );
}

void pcl::cuda::CudaData::download(CloudData& cloud_data) const
{
  cloud_data.create(rows_, colsBytes_);

  if (!pitched())
    cudaSafeCall( cudaMemcpy(cloud_data.ptr<void>(), data_, colsBytes_, cudaMemcpyDeviceToHost) );
  else
    cudaSafeCall( cudaMemcpy2D(cloud_data.ptr<void>(), cloud_data.step(), data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToHost) );  
}         
 
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////
// CudaCloudSet

pcl::cuda::CudaCloudSet::CudaCloudSet()
{
  for(int i = 0; i < MAX_CLOUDS; ++i)
    storage_[i].type_ = ChannelKind::None;
}

pcl::cuda::CudaData pcl::cuda::CudaCloudSet::create(size_type rows, size_type colsBytes, const std::string& name)
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

void pcl::cuda::CudaCloudSet::set(const CudaData& data, const std::string& name)
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

void pcl::cuda::CudaCloudSet::remove(const std::string& name)
{
  int i = name_search(name);  
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_.release();
    storage_[i].name_.clear();
    storage_[i].type_ = ChannelKind::None;
  }
}

pcl::cuda::CudaData pcl::cuda::CudaCloudSet::get(const std::string& name)
{
  int i = name_search(name);
  return i == MAX_CLOUDS ? CloudData() : storage_[i].data_;
}

const pcl::cuda::CudaData pcl::cuda::CudaCloudSet::get(const std::string& name) const
{
  int i = name_search(name);
  return i == MAX_CLOUDS ? CloudData() : storage_[i].data_;
}
                        
void pcl::cuda::CudaCloudSet::set(const CudaData& cloud_data, int type)
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
  
void pcl::cuda::CudaCloudSet::remove(int type)
{
  int i = type_search(type);
  if (i != MAX_CLOUDS)
  {
    storage_[i].data_.release();
    storage_[i].name_.clear();
    storage_[i].type_ = ChannelKind::None;
  }
}
   
int pcl::cuda::CudaCloudSet::name_search(const std::string& name) const
{
  int i = 0;  
  for(; i < MAX_CLOUDS; ++i)  
    if(storage_[i].name_ == name)
      break;
  return i;
}

int pcl::cuda::CudaCloudSet::type_search(int type) const
{
  int i = 0;  
  for(; i < MAX_CLOUDS; ++i)  
    if(storage_[i].type_ == type)
      break;
  return i;
}

int pcl::cuda::CudaCloudSet::empty_search() const
{
  int i = 0;  
  for(; i < MAX_CLOUDS; ++i)  
    if(storage_[i].name_.empty() && storage_[i].type_ ==  ChannelKind::None)
      break;
  return i;
}


