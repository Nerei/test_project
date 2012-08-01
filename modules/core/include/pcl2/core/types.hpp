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

#include <cassert>
#include <pcl2/core/pcl_exports.hpp>
#include <Eigen/Geometry>


namespace pcl
{

  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// <cstdint.hpp> //////////////////////////////////////////////

#if defined _MSC_VER || defined __BORLANDC__
  typedef __int64 int64_t;
  typedef unsigned __int64 uint64_t;
#endif

  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Auxilary types /////////////////////////////////////////////

  /** \brief @b Point3<T> class */
  template<typename T> struct PCL_EXPORTS Point3
  {
    union
    {
      struct { T x, y, z; };
      T data[3];
    };

    Point3() : x(T()), y(T()), z(T()) {}
    Point3(T vx, T vy, T vz) : x(vx), y(vy), z(vz) {};

    T  operator[](int i) const { return data[i]; }
    T& operator[](int i)       { return data[i]; }
  };

  /** \brief @b Point3<T> class */
  template<typename T> struct PCL_EXPORTS Point3a
  {
    union
    {
      struct { T x, y, z, w; };
      T data[4];
    };

    Point3a() : x(T()), y(T()), z(T()) {}
    Point3a(T vx, T vy, T vz) : x(vx), y(vy), z(vz) {};

    T  operator[](int i) const { return data[i]; }
    T& operator[](int i)       { return data[i]; }
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Interface types ////////////////////////////////////////////

  /** \brief @b Point3f class */
  struct Point3f : public Point3<float>
  {
    Point3f() {}
    Point3f(float x, float y, float z) : Point3<float>(x, y, z) {}
  };

  /** \brief @b Point3d class */
  struct Point3d : public Point3<double>
  {
    Point3d() {}
    Point3d(double x, double y, double z) : Point3<double>(x, y, z) {}
  };

  /** \brief @b Point3fa class */
  struct Point3fa : public Point3a<float>
  {
    Point3fa() {}
    Point3fa(float x, float y, float z) : Point3a<float>(x, y, z) {}
  };

  /** \brief @b Normal3f class */
  struct Normal3f : public Point3<float>
  {
    Normal3f() {}
    Normal3f(float x, float y, float z) : Point3<float>(x, y, z) {}
  };

  /** \brief @b Point3fa class */
  struct Normal3fa : public Point3a<float>
  {
    Normal3fa() {}
    Normal3fa(float x, float y, float z) : Point3a<float>(x, y, z) {}
  };

  /** \brief @b Normal3f class */
  struct Normal3d : public Point3<double>
  {
    Normal3d() {}
    Normal3d(double x, double y, double z) : Point3<double>(x, y, z) {}
  };

  /** \brief @b RGB class  */
  struct PCL_EXPORTS RGB
  {
    typedef unsigned char value_type;

    union
    {
      unsigned char data[3];
      struct { unsigned char r, g, b; };
    };

    RGB() : r(0), g(0), b(0) {}
    RGB(unsigned char red, unsigned char green, unsigned char blue) : r(red), g(green), b(blue) {}

    unsigned char  operator[](int i) const { return data[i]; }
    unsigned char& operator[](int i)       { return data[i]; }
  };

  /** \brief @b RGBA class */
  struct PCL_EXPORTS RGBA
  {
    typedef unsigned char value_type;

    union
    {
      int rgba;
      unsigned char data[4];
      struct { unsigned char r, g, b, a; };
    };
    RGBA() : rgba(0) {};
    RGBA(int rgba_) : rgba(rgba_) {};
    RGBA(unsigned char red, unsigned char green, unsigned char blue) : r(red), g(green), b(blue), a(0) {}

    unsigned char  operator[](int i) const { return data[i]; }
    unsigned char& operator[](int i)       { return data[i]; }
  };

  //////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////// Borrow from Eigen  ///////////////////////////////////////////

#define PCL_NOT_ALIGNED_EIGEN_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)                      \
  typedef Eigen::Matrix<Type, Size, Size, (Eigen::DontAlign|Eigen::RowMajor)>    Matrix##SizeSuffix##TypeSuffix;  \
  typedef Eigen::Matrix<Type, Size,    1, (Eigen::DontAlign                )>    Vector##SizeSuffix##TypeSuffix;  \
  typedef Eigen::Matrix<Type,    1, Size, (Eigen::DontAlign|Eigen::RowMajor)> RowVector##SizeSuffix##TypeSuffix;  \
  typedef Eigen::Array<Type,  Size,    1, (Eigen::DontAlign                )>     Array##SizeSuffix##TypeSuffix;  \
  typedef Eigen::Array<Type,  Size, Size, (Eigen::DontAlign|Eigen::RowMajor)>     Array##SizeSuffix##SizeSuffix##TypeSuffix;

#define PCL_NOT_ALIGNED_EIGEN_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
  PCL_NOT_ALIGNED_EIGEN_TYPEDEFS(Type, TypeSuffix, 2, 2) \
  PCL_NOT_ALIGNED_EIGEN_TYPEDEFS(Type, TypeSuffix, 3, 3) \
  PCL_NOT_ALIGNED_EIGEN_TYPEDEFS(Type, TypeSuffix, 4, 4)
  
  PCL_NOT_ALIGNED_EIGEN_TYPEDEFS_ALL_SIZES(int,    i)
  PCL_NOT_ALIGNED_EIGEN_TYPEDEFS_ALL_SIZES(float,  f)
  PCL_NOT_ALIGNED_EIGEN_TYPEDEFS_ALL_SIZES(double, d)

#undef PCL_NOT_ALIGNED_EIGEN_TYPEDEFS
#undef PCL_NOT_ALIGNED_EIGEN_TYPEDEFS_ALL_SIZES

  typedef Eigen::Transform<float,  2, Eigen::Affine, (Eigen::DontAlign|Eigen::RowMajor)> Affine2f;
  typedef Eigen::Transform<float,  3, Eigen::Affine, (Eigen::DontAlign|Eigen::RowMajor)> Affine3f;
  typedef Eigen::Transform<double, 2, Eigen::Affine, (Eigen::DontAlign|Eigen::RowMajor)> Affine2d;
  typedef Eigen::Transform<double, 3, Eigen::Affine, (Eigen::DontAlign|Eigen::RowMajor)> Affine3d;

  typedef Eigen::Quaternion<float,  Eigen::DontAlign> Quaternionf;
  typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaterniond;

  using Eigen::AngleAxisf;
  using Eigen::AngleAxisd;
  using Eigen::Rotation2Df;
  using Eigen::Rotation2Dd;
  using Eigen::Translation2f;
  using Eigen::Translation2d;
  using Eigen::Translation3f;
  using Eigen::Translation3d;

  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Traits types ////////////////////////////////////////////

  /** \brief @b ChannelKind class */
  struct ChannelKind
  {
    enum
    {
      None,

      Byte,

      Point3f, Point3fa, Point3d,    // points
      Normal3f, Normal3fa, Normal3d, // normals
      
      RGB, RGBA, //colors

      Vector3f, Vector3d, Vector4f, Vector4d // Eigen types
    };

    static size_t size_of(int type);
  };

  template<typename T> struct channel_traits;
  template<> struct channel_traits<void> { enum { type = ChannelKind::None }; };
  template<> struct channel_traits<unsigned char> { enum { type = ChannelKind::Byte }; };

#define DECLARE_CHANNEL_TRAITS(type_name) \
  template<> struct channel_traits<type_name> { enum { type = ChannelKind::type_name }; };

  DECLARE_CHANNEL_TRAITS(Point3f);
  DECLARE_CHANNEL_TRAITS(Point3fa);
  DECLARE_CHANNEL_TRAITS(Point3d);
  DECLARE_CHANNEL_TRAITS(Normal3f);
  DECLARE_CHANNEL_TRAITS(Normal3fa);
  DECLARE_CHANNEL_TRAITS(Normal3d);
  DECLARE_CHANNEL_TRAITS(RGB);
  DECLARE_CHANNEL_TRAITS(RGBA);

  DECLARE_CHANNEL_TRAITS(Vector3f);
  DECLARE_CHANNEL_TRAITS(Vector3d);
  DECLARE_CHANNEL_TRAITS(Vector4f);
  DECLARE_CHANNEL_TRAITS(Vector4d);

#undef DECLARE_CHANNEL_TRAITS


  /** \brief @b MemoryKind class */
  struct MemoryKind
  {
    enum
    {
      CPU, CUDA, OCL,
      CUDA_ZERO_COPY // for notebooks and tablets, where CPU & GPU memory is phisically the same chip.
    };
  };


  template<typename T> struct memkind_traits;

#define DECLARE_MEMORY_TRAITS(type, mem) \
  template<> struct memkind_traits<type> { enum { kind = MemoryKind::mem }; }

#define DECLARE_TEMPLATED_MEMORY_TRAITS(type, mem) \
  template<typename T> struct memkind_traits<type> { enum { kind = MemoryKind::mem }; }

//  DECLARE_TEMPLATED_MEMORY_TRAITS(std::vector<T>, CPU);
//  DECLARE_TEMPLATED_MEMORY_TRAITS(pcl::Cloud<T>, CPU);
//  DECLARE_MEMORY_TRAITS(pcl::CloudData, CPU);
//  DECLARE_MEMORY_TRAITS(pcl::CloudSet, CPU);

//  DECLARE_TEMPLATED_MEMORY_TRAITS(pcl::cuda::CudaCloud<T>, CUDA);
//  DECLARE_MEMORY_TRAITS(pcl::cuda::CudaData, CUDA);
//  DECLARE_MEMORY_TRAITS(pcl::cuda::CudaCloudSet, CUDA);


#undef DECLARE_MEMORY_TRAITS
#undef DECLARE_TEMPLATED_MEMORY_TRAITS


}

