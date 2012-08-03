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



#include <pcl2/core/core.hpp>

#include<iostream>

using namespace std;

int main()
{
    pcl::Cloud<int> c(10, 10);

    pcl::cuda::CudaData a(10, 10);
    pcl::cuda::CudaCloud<int> g(10, 10);

    g.upload(c);
    cout << "hello" << endl;


#define DECLARE_TYPES(Type, TypeSuffix, Size, SizeSuffix)       \
  pcl::Matrix##SizeSuffix##TypeSuffix m##SizeSuffix##TypeSuffix;     \
  pcl::Vector##SizeSuffix##TypeSuffix v##SizeSuffix##TypeSuffix;     \
  pcl::RowVector##SizeSuffix##TypeSuffix rv##SizeSuffix##TypeSuffix; \
  pcl::Array##SizeSuffix##TypeSuffix a##SizeSuffix##TypeSuffix;      \
  pcl::Array##SizeSuffix##SizeSuffix##TypeSuffix aa##SizeSuffix##SizeSuffix##TypeSuffix;

#define DECLARE_TYPES_ALL_SIZES(Type, TypeSuffix) \
  DECLARE_TYPES(Type, TypeSuffix, 2, 2) \
  DECLARE_TYPES(Type, TypeSuffix, 3, 3) \
  DECLARE_TYPES(Type, TypeSuffix, 4, 4)

  DECLARE_TYPES_ALL_SIZES(int,    i)
  DECLARE_TYPES_ALL_SIZES(float,  f)
  DECLARE_TYPES_ALL_SIZES(double, d)

  pcl::Affine2f aff2f;
  pcl::Affine3f aff3f;
  pcl::Affine2d aff2d;
  pcl::Affine3d aff3d;

  pcl::Quaternionf qf;
  pcl::Quaterniond qd;

  pcl::AngleAxisf anf;
  pcl::AngleAxisd andf;
  pcl::Rotation2Df r2f(3);
  pcl::Rotation2Dd r2d(3);
  pcl::Translation2f t2f;
  pcl::Translation2d t2d;
  pcl::Translation3f t3f;
  pcl::Translation3d t3d;
}
