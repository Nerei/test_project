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


#if !defined PCLAPI_EXPORTS
  #error "Header <pcl2/core/internal.hpp> is only for internal use in PCL "
#else

#include <pcl2/pcl_config.h>

#if defined(HAVE_TBB)
  #include <tbb/tbb.h>
#endif

namespace pcl
{
  namespace internal
  {
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // TBB helpers

#if defined(HAVE_TBB)
    using tbb::blocked_range;
    using tbb::split;
    
    using tbb::parallel_for;
    using tbb::parallel_do;
    using tbb::parallel_reduce;

#else

    template<typename T> 
    class blocked_range
    {
    public:
      typedef T const_iterator;
      typedef std::size_t size_type;

      blocked_range() {}
      blocked_range( T beg, T end, size_type grainsize = 1) : end_(end), beg_(beg_), grainsize_(grainsize) {}

      const_iterator begin() const { return beg_; }
      const_iterator end()   const { return end_; }
      size_type size() const { return size_type(end_ - beg_); }
      size_type grainsize() const { return grainsize_; }
      bool empty() const { return !(beg_ < end_);}
      bool is_divisible() const {return grainsize_ < size();}

    private:
      T end_;
      T beg_;
      size_type grainsize_;
    };

    class split {};

    template<typename Range, typename Body> static inline
    void parallel_for( const Range& range, const Body& body )
    {
      body(range);
    }
    
    template<typename Iterator, typename Body> static inline
    void parallel_do( Iterator first, Iterator last, const Body& body )
    {
      for( ; first != last; ++first )
        body(*first);
    }

    template<typename Range, typename Body> static inline
    void parallel_reduce( const Range& range, Body& body )
    {
      body(range);
    }
#endif
  }

  namespace parallel = pcl::internal;
}

#endif
