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

#include "precomp.hpp"

using namespace std;

void pcl::fatal_error(const std::string& message, const std::string& file, int line, const std::string& /*func*/)
{    
  cout << "Fatal error: " << message << " in " << file << ":" << line << endl;
  exit(0);
}




void pcl::scaleCloud1(const Cloud& input_cloud, Cloud& output_cloud)
{
  //gets smart pointer for "typed" data
  const Channel<Point3f> channel = input_cloud.get<Point3f>();

  //gets smart pointer ChannelData for "named" "untyped" channel and reinterprets it as "typed" Channel
  const Channel<float> fpfh33f(input_cloud.get("fpfh33"));

  PCL_Assert( fpfh33f.cols() == 33 );
  size_t features_count = fpfh33f.rows();

  //allocates "typed" channel and gets smart pointer to it
  Channel<Point3f> output = output_cloud.create<Point3f>(1, 10);

  //allocates "named" "untyped" channel and gets smart pointer to it
  ChannelData output_featues = output_cloud.create(features_count, 33 * sizeof(float), "fpfh33");

  //rewrites "named" channel with new data, old buffer will be released
  output_cloud.set(fpfh33f, "fpfh33");
}

void pcl::scaleCloud2(const In& input_cloud, Out& output_cloud)
{
  //gets named data
  const ChannelData channel_data = input_cloud.get("fpfh33");

  //gets named data and reinterprets as typed Channel
  const Channel<float> channel = input_cloud.get<float>("fpfh33");

  //checks that data is the same
  PCL_Assert(channel.ptr() == channel_data.ptr<float>());

  //gets data, works only if 'input_Channel' proxy class was constructed with Channel or ChannelData
  const ChannelData channel_data1 = input_cloud.get();

  //gets "typed" data, works for Cloud, Channel<Normal3f>, std::vector<Normal3f>
  //returns empty pointer if input doesn't contain such channel
  const Channel<Normal3f> channel1 = input_cloud.get<Normal3f>();

  size_t rows = 10;
  size_t cols = 10;
  size_t colsBytes = cols * sizeof(Point3f);

  //allocates "typed" channel, works for Cloud, Channel<Point3f>, vector<Point3f>
  Channel<Point3f> c1 = output_cloud.create<Point3f>(rows, cols);

  //allocates "named" channel, works only for Cloud
  ChannelData c2 = output_cloud.create(rows, colsBytes, "named");

  // allocates array, works only for Channel<T>, ChannelData, vector<T>
  // won't work for Cloud since "unnamed" and "untyped"
  // PCL_Assert(colsBytes % sizeof(T) == 0), where T is a value of template parameter of object from which output_Channel was constructed
  // That means if passed vector<Point3f>, then can't allocate colsBytes == 2.    
  ChannelData c3 = output_cloud.create(rows, colsBytes);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// pc::ChannelKind

size_t pcl::ChannelKind::size_of(int type)
{
  size_t sizes[] = 
  {
    0, //None,
    sizeof(unsigned char), //Byte,

    sizeof(Point3f), sizeof(Point3fa), sizeof(Point3d),    // points
    sizeof(Normal3f), sizeof(Normal3fa), sizeof(Normal3d), // normals

    sizeof(RGB), sizeof(RGBA), //colors

    sizeof(Vector3f), sizeof(Vector3d), sizeof(Vector4f), sizeof(Vector4d) // Eigen types
  };

  const size_t sizes_size = sizeof(sizes)/sizeof(sizes[0]);

  PCL_Assert(0U <= type && type < sizes_size);
  return sizes[type];
}


/////////////////////////////////////////////////////////////////////////////////////////////////
// pc::In

pcl::In::In() : kind_(Kind::None), type_(channel_traits<void>::type), obj_(0) {}
template<> pcl::In::In(const ChannelData& Channel_data) : kind_(Kind::Channel), type_(channel_traits<unsigned char>::type), obj_((void*)&Channel_data) {}
template<> pcl::In::In(const Cloud& Channel_set)   : kind_(Kind::Cloud),  type_(channel_traits<void>::type), obj_((void*)&Channel_set) {}

int pcl::In::type() const { return type_; }

const pcl::ChannelData pcl::In::get(const std::string& name) const
{
  PCL_Assert(kind_ == Kind::Cloud);
  return reinterpret_cast<const Cloud*>(obj_)->get(name);;
}

const pcl::ChannelData pcl::In::get() const
{
  if (kind_ == Kind::Cloud)
    PCL_Assert(!"Can't retrieve untyped and unnamed ChannelData from Cloud");
  if (kind_ == Kind::Channel)
    return *reinterpret_cast<const ChannelData*>(obj_);
  else /* kind_ == KInd::StdVector */
    return from_vector();
  
  return ChannelData();
};

namespace 
{
  template<typename T> inline const void* get_vecdata(const void *obj, size_t& sizeBytes, bool allocate) 
  {  
     if (allocate)
     { 
       PCL_Assert( sizeBytes % sizeof(T) == 0);
       std::vector<T>& v = *(std::vector<T>*)obj;
       v.resize(sizeBytes/sizeof(T));
     }

     const std::vector<T>& v = *reinterpret_cast<const std::vector<T>*>(obj); 
     sizeBytes = v.size() * sizeof(T);
     return &v[0];
  }
}
  
pcl::ChannelData pcl::In::from_vector(size_t size_bytes, bool allocate) const
{
  typedef const void* (*func_t)(const void *, size_t&, bool);
  using namespace pcl;

  func_t funcs[] = 
  { 
    0, // None
    get_vecdata<unsigned char>, //Byte
    get_vecdata<Point3f>, get_vecdata<Point3fa>, get_vecdata<Point3d>, //points
    get_vecdata<Normal3f>, get_vecdata<Normal3fa>, get_vecdata<Normal3d>, //normals

    get_vecdata<RGB>, get_vecdata<RGBA>, // colors

    get_vecdata<Vector3f>, get_vecdata<Vector3d>, get_vecdata<Vector4f>, get_vecdata<Vector4d>, // Eigen types
  };

  size_t funcs_size = sizeof(funcs)/sizeof(funcs[0]);
  if (type_ == 0 || type_ >= (int)funcs_size)
      PCL_FatalError("Unsupported type");

  const void *data = funcs[type_](obj_, size_bytes, allocate);
  return ChannelData(1, size_bytes, (void*)data);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
// pc::Out

template<> pcl::Out::Out(const ChannelData& Channel_data)  : In(Channel_data) {}
template<> pcl::Out::Out(const Cloud& Channel_set) : In(Channel_set) {}

pcl::ChannelData pcl::Out::create(size_type rows, size_type colsBytes, const std::string& name)
{
  PCL_Assert(kind_ == Kind::Cloud);
  Cloud& Channel_set = *reinterpret_cast<Cloud*>(obj_);
  return Channel_set.create(rows, colsBytes, name);
}

pcl::ChannelData pcl::Out::create(size_type rows, size_type colsBytes)
{
  if (kind_ == Kind::Cloud)
    PCL_FatalError("Can't allocate unnamed and untyped data in Cloud");
  
  size_t type_size = ChannelKind::size_of(type_);
  PCL_Assert(colsBytes % type_size == 0);
  
  if (kind_ == Kind::Channel)
  {
    ChannelData Channel_data = *reinterpret_cast<ChannelData*>(obj_);
    Channel_data.create(rows, colsBytes);
    return Channel_data;
  }
  else /* Kind::StdVector */
    return from_vector(colsBytes * rows, true);
}

