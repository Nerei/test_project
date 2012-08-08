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

#include <pcl2/pcl_config.h>
#include <pcl2/core/clouds/clouds.hpp>
#include <pcl2/core/clouds/cuda_clouds.hpp>
#include <pcl2/core/core.hpp>
#include <pcl2/core/internal.hpp>
#include <algorithm>
#include <iostream>

#if defined HAVE_CUDA
  #include <pcl2/core/cuda/safe_call.hpp>
#endif

#if defined _MSC_VER
  #pragma warning (disable : 4996)
#endif

//////////////////////////    XADD    ///////////////////////////////

#if defined (__GNUC__)
    
    #if __GNUC__*10 + __GNUC_MINOR__ >= 42

        #if !defined WIN32 && (defined __i486__ || defined __i586__ || defined __i686__ || defined __MMX__ || defined __SSE__  || defined __ppc__)
            #define __XADD__ __sync_fetch_and_add
        #else
            #include <ext/atomicity.h>
            #define __XADD__ __gnu_cxx::__exchange_and_add
        #endif
    #else
        #include <bits/atomicity.h>
        #if __GNUC__*10 + __GNUC_MINOR__ >= 34
            #define __XADD__ __gnu_cxx::__exchange_and_add
        #else
            #define __XADD__ __exchange_and_add
        #endif
  #endif
    
#elif defined WIN32 || defined _WIN32
    #include <intrin.h>
    #define __XADD__(addr,delta) _InterlockedExchangeAdd((long volatile*)(addr), (delta))
#else
    template<typename _Tp> static inline _Tp __XADD__(_Tp* addr, _Tp delta)
    { int tmp = *addr; *addr += delta; return tmp; }   
#endif


