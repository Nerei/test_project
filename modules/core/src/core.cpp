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




void pcl::scaleCloud1(const CloudSet& input_cloud, CloudSet& output_cloud)
{
  //gets smart pointer for "typed" data
  const Cloud<Point3f> cloud = input_cloud.get<Point3f>();

  //gets smart pointer CloudData for "named" "untyped" channel and reinterprets it as "typed" cloud
  const Cloud<float> fpfh33f(input_cloud.get("fpfh33"));

  PCL_Assert( fpfh33f.cols() == 33 );
  size_t features_count = fpfh33f.rows();

  //allocates "typed" channel and gets smart pointer to it
  Cloud<Point3f> output = output_cloud.create<Point3f>(1, 10);

  //allocates "named" "untyped" channel and gets smart pointer to it
  CloudData output_featues = output_cloud.create(features_count, 33 * sizeof(float), "fpfh33");

  //rewrites "named" channel with new data, old buffer will be released
  output_cloud.set(fpfh33f, "fpfh33");
}

void pcl::scaleCloud2(const In& input_cloud, Out& output_cloud)
{
  //gets named data
  const CloudData cloud_data = input_cloud.get("fpfh33");

  //gets named data and reinterprets as typed cloud
  const Cloud<float> cloud = input_cloud.get<float>("fpfh33");

  //checks that data is the same
  PCL_Assert(cloud.ptr() == cloud_data.ptr<float>());

  //gets data, works only if 'input_cloud' proxy class was constructed with Cloud or CloudData
  const CloudData cloud_data1 = input_cloud.get();

  //gets "typed" data, worsk for CloudSet, Cloud<Normal3f>, std::vector<Normal3f>
  //returns empty pointer of input doesn't contain such channel
  const Cloud<Normal3f> cloud1 = input_cloud.get<Normal3f>();

  //allocates "typed" channel, works for CloudSet, Cloud<Point3f>, vector<Point3f>
  Cloud<Point3f> c1 = output_cloud.create<Point3f>(10, 10);

  //allocates "named" channel, works only for CloudSet
  CloudData c2 = output_cloud.create(10, 10 * sizeof(Point3f), "named");

  // allocates array, works only for Cloud<T>, CloudData, vector<T>
  // PCL_Assert(10 * sizeof(Point3f) % sizeof(T) == 0)
  // won't work for CloudSet since "unnamed" and "untyped"
  CloudData c3 = output_cloud.create(10, 10 * sizeof(Point3f));

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
template<> pcl::In::In(const CloudData& cloud_data) : kind_(Kind::Cloud), type_(channel_traits<unsigned char>::type), obj_((void*)&cloud_data) {}
template<> pcl::In::In(const CloudSet& cloud_set)   : kind_(Kind::CloudSet),  type_(channel_traits<void>::type), obj_((void*)&cloud_set) {}

int pcl::In::type() const { return type_; }

const pcl::CloudData pcl::In::get(const std::string& name) const
{
  PCL_Assert(kind_ == Kind::CloudSet);
  return reinterpret_cast<const CloudSet*>(obj_)->get(name);;
}

const pcl::CloudData pcl::In::get() const
{
  if (kind_ == Kind::CloudSet)
    PCL_Assert(!"Can't retrieve untyped and unnamed CloudData from CloudSet");
  if (kind_ == Kind::Cloud)
    return *reinterpret_cast<const CloudData*>(obj_);
  else /* kind_ == KInd::StdVector */
    return from_vector();
  
  return CloudData();
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
  
pcl::CloudData pcl::In::from_vector(size_t size_bytes, bool allocate) const
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
  return CloudData(1, size_bytes, (void*)data);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
// pc::Out

template<> pcl::Out::Out(const CloudData& cloud_data)  : In(cloud_data) {}
template<> pcl::Out::Out(const CloudSet& cloud_set) : In(cloud_set) {}

pcl::CloudData pcl::Out::create(size_type rows, size_type colsBytes, const std::string& name)
{
  PCL_Assert(kind_ == Kind::CloudSet);
  CloudSet& cloud_set = *reinterpret_cast<CloudSet*>(obj_);
  return cloud_set.create(rows, colsBytes, name);
}

pcl::CloudData pcl::Out::create(size_type rows, size_type colsBytes)
{
  if (kind_ == Kind::CloudSet)
    PCL_FatalError("Can't allocate unnamed and untyped data in CloudSet");
  
  size_t type_size = ChannelKind::size_of(type_);
  PCL_Assert(colsBytes % type_size == 0);
  
  if (kind_ == Kind::Cloud)
  {
    CloudData cloud_data = *reinterpret_cast<CloudData*>(obj_);
    cloud_data.create(rows, colsBytes);
    return cloud_data;
  }
  else /* Kind::StdVector */
    return from_vector(colsBytes * rows, true);
}

