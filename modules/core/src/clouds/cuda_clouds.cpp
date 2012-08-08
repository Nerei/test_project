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
// CudaChannelData

#if !defined HAVE_CUDA

void throw_nocuda() { PCL_FatalError("The library was compiled without CUDA support"); }

pcl::cuda::CudaChannelData::CudaChannelData() { throw_nocuda(); }       
pcl::cuda::CudaChannelData::~CudaChannelData() { throw_nocuda(); }       
pcl::cuda::CudaChannelData::CudaChannelData(size_type /*sizeBytes*/) { throw_nocuda(); }
pcl::cuda::CudaChannelData::CudaChannelData(size_type /*rows*/, size_type /*colsBytes*/) { throw_nocuda(); }
pcl::cuda::CudaChannelData::CudaChannelData(size_type /*sizeBytes*/, void* /*data*/) { throw_nocuda(); }
pcl::cuda::CudaChannelData::CudaChannelData(const CudaChannelData& /*other*/) { throw_nocuda(); }
pcl::cuda::CudaChannelData& pcl::cuda::CudaChannelData::operator=(const CudaChannelData& /*other*/) { throw_nocuda(); return *this; }
void pcl::cuda::CudaChannelData::create(size_t /*sizeBytes*/) { throw_nocuda(); }
void pcl::cuda::CudaChannelData::release() { throw_nocuda(); }
void pcl::cuda::CudaChannelData::copyTo(CudaChannelData& /*other*/) const { throw_nocuda(); }
void pcl::cuda::CudaChannelData::swap(CudaChannelData& other) { throw_nocuda(); }
bool pcl::cuda::CudaChannelData::empty() const { throw_nocuda(); return true; }          
pcl::ChannelData::size_type pcl::cuda::CudaChannelData::colsBytes() const { throw_nocuda(); return 0; }

void pcl::cuda::CudaChannelData::upload(const ChannelData& /*Channel_data*/) { throw_nocuda(); }
void pcl::cuda::CudaChannelData::download(ChannelData& /*Channel_data*/) const { throw_nocuda(); }

#else

pcl::cuda::CudaChannelData::CudaChannelData() : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) {} 
pcl::cuda::CudaChannelData::CudaChannelData(size_type sizeBytes) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) 
{ create(sizeBytes); } 

pcl::cuda::CudaChannelData::CudaChannelData(size_type rows, size_type colsBytes) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true)
{ create(rows, colsBytes); } 

pcl::cuda::CudaChannelData::CudaChannelData(size_type sizeBytes, void *data) : data_((unsigned char*)data), step_(0), rows_(1), colsBytes_(sizeBytes), refcount_(0), dense_(true) {}

pcl::cuda::CudaChannelData::CudaChannelData(size_type rows, size_type colsBytes, void *data, size_type step) 
   : data_((unsigned char*)data), step_(step), rows_(rows), colsBytes_(colsBytes), refcount_(0), dense_(true) {}
          
pcl::cuda::CudaChannelData::~CudaChannelData() { release(); }


pcl::cuda::CudaChannelData::CudaChannelData(const CudaChannelData& other) : data_(other.data_), step_(other.step_), rows_(other.rows_), 
    colsBytes_(other.colsBytes_), refcount_(other.refcount_), dense_(other.dense_)
{
  if( refcount_ )
    __XADD__(refcount_, 1);
}

pcl::cuda::CudaChannelData::CudaChannelData(const ChannelData& other) : data_(0), step_(0), rows_(0), colsBytes_(0), refcount_(0), dense_(true) 
{ upload(other); } 

pcl::cuda::CudaChannelData& pcl::cuda::CudaChannelData::operator=(const CudaChannelData& other)
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
 
void pcl::cuda::CudaChannelData::create(size_t sizeBytes) 
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

void pcl::cuda::CudaChannelData::create(size_type rows, size_type colsBytes)
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
          
void pcl::cuda::CudaChannelData::release()
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

void pcl::cuda::CudaChannelData::swap(CudaChannelData& other)
{
  std::swap(data_, other.data_);
  std::swap(step_, other.step_);
  std::swap(rows_, other.rows_);
  std::swap(colsBytes_, other.colsBytes_);
  std::swap(refcount_, other.refcount_);
  std::swap(dense_, other.dense_);
}

void pcl::cuda::CudaChannelData::copyTo(CudaChannelData& other) const
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

bool pcl::cuda::CudaChannelData::empty() const { return !data_; }
pcl::cuda::CudaChannelData::size_type pcl::cuda::CudaChannelData::rows() const { return rows_; }
pcl::cuda::CudaChannelData::size_type pcl::cuda::CudaChannelData::colsBytes() const { return colsBytes_; }
pcl::cuda::CudaChannelData::size_type pcl::cuda::CudaChannelData::step() const { return step_; }
bool& pcl::cuda::CudaChannelData::dense() { return dense_; }
bool  pcl::cuda::CudaChannelData::dense() const  { return dense_; }  
bool  pcl::cuda::CudaChannelData::pitched() const { return step() != 0; }

void pcl::cuda::CudaChannelData::upload(const ChannelData& Channel_data)
{
  if (Channel_data.rows() == 1)
  {
    create(Channel_data.colsBytes());
    cudaSafeCall( cudaMemcpy(data_, Channel_data.ptr<void>(), colsBytes_, cudaMemcpyHostToDevice) );      
  }
  else
  {
    create(Channel_data.rows(), Channel_data.colsBytes());
    cudaSafeCall( cudaMemcpy2D(data_, step_, Channel_data.ptr<void>(), Channel_data.step(), colsBytes_, rows_, cudaMemcpyHostToDevice) );  
  }
  cudaSafeCall( cudaDeviceSynchronize() );
}

void pcl::cuda::CudaChannelData::download(ChannelData& Channel_data) const
{
  Channel_data.create(rows_, colsBytes_);

  if (!pitched())
    cudaSafeCall( cudaMemcpy(Channel_data.ptr<void>(), data_, colsBytes_, cudaMemcpyDeviceToHost) );
  else
    cudaSafeCall( cudaMemcpy2D(Channel_data.ptr<void>(), Channel_data.step(), data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToHost) );  
}         
 
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////
// CudaCloud

pcl::cuda::CudaCloud::CudaCloud()
{
  for(int i = 0; i < MAX_ChannelS; ++i)
    storage_[i].type_ = ChannelKind::None;
}

pcl::cuda::CudaChannelData pcl::cuda::CudaCloud::create(size_type rows, size_type colsBytes, const std::string& name)
{
  int i = name_search(name);
  if (i != MAX_ChannelS)
  {
    storage_[i].data_.create(rows, colsBytes);
    storage_[i].type_ = ChannelKind::None;
    return storage_[i].data_;
  }

  i = empty_search();
  if (i != MAX_ChannelS)
  {
    storage_[i].data_.create(rows, colsBytes);
    storage_[i].name_ = name;
    storage_[i].type_ = ChannelKind::None;
  }
  else
      PCL_FatalError("Maximal supported Channels limit reached for Cloud");
  return storage_[i].data_;
}

void pcl::cuda::CudaCloud::set(const CudaChannelData& data, const std::string& name)
{
  int i = name_search(name);  
  
  if (i != MAX_ChannelS)
  {
    storage_[i].data_ = data;
    return;
  } 

  i = empty_search();
  if (i != MAX_ChannelS)
  {
    storage_[i].data_ = data;
    storage_[i].name_ = name;
    storage_[i].type_ = ChannelKind::None;
  }
  else
      PCL_FatalError("Maximal supported Channels limit reached for Cloud");
}     

void pcl::cuda::CudaCloud::remove(const std::string& name)
{
  int i = name_search(name);  
  if (i != MAX_ChannelS)
  {
    storage_[i].data_.release();
    storage_[i].name_.clear();
    storage_[i].type_ = ChannelKind::None;
  }
}

pcl::cuda::CudaChannelData pcl::cuda::CudaCloud::get(const std::string& name)
{
  int i = name_search(name);
  return i == MAX_ChannelS ? ChannelData() : storage_[i].data_;
}

const pcl::cuda::CudaChannelData pcl::cuda::CudaCloud::get(const std::string& name) const
{
  int i = name_search(name);
  return i == MAX_ChannelS ? ChannelData() : storage_[i].data_;
}
                        
void pcl::cuda::CudaCloud::set(const CudaChannelData& Channel_data, int type)
{
  int i = type_search(type);
  if (i != MAX_ChannelS)
  {
    storage_[i].data_ = Channel_data;    
    return;
  } 

  i = empty_search();
  if (i != MAX_ChannelS)
  {
    storage_[i].data_ = Channel_data;
    storage_[i].type_ = type; 
    storage_[i].name_.clear();    
  }
  else
      PCL_FatalError("Maximal supported Channels limit reached for Cloud");
}
  
void pcl::cuda::CudaCloud::remove(int type)
{
  int i = type_search(type);
  if (i != MAX_ChannelS)
  {
    storage_[i].data_.release();
    storage_[i].name_.clear();
    storage_[i].type_ = ChannelKind::None;
  }
}
   
int pcl::cuda::CudaCloud::name_search(const std::string& name) const
{
  int i = 0;  
  for(; i < MAX_ChannelS; ++i)  
    if(storage_[i].name_ == name)
      break;
  return i;
}

int pcl::cuda::CudaCloud::type_search(int type) const
{
  int i = 0;  
  for(; i < MAX_ChannelS; ++i)  
    if(storage_[i].type_ == type)
      break;
  return i;
}

int pcl::cuda::CudaCloud::empty_search() const
{
  int i = 0;  
  for(; i < MAX_ChannelS; ++i)  
    if(storage_[i].name_.empty() && storage_[i].type_ ==  ChannelKind::None)
      break;
  return i;
}


