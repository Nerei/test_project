#include "precomp.hpp"

#if defined WIN32 || defined WINCE
  #if !defined (_WIN32_WINNT)    // This is needed for the declaration of TryEnterCriticalSection in winbase.h with Visual Studio 2005 (and older?)
    #define _WIN32_WINNT 0x0400  // http://msdn.microsoft.com/en-us/library/ms686857(VS.85).aspx
  #endif

  #include <windows.h>
  #undef small
  #undef min
  #undef max
#endif

namespace
{

#if defined _MSC_VER || defined __BORLANDC__
  typedef __int64 int64_t;
  typedef unsigned __int64 uint64_t;
#endif

  int64_t getTickCount(void)
  {

#if defined WIN32 || defined _WIN32 || defined WINCE
    LARGE_INTEGER counter;
    QueryPerformanceCounter( &counter );
    return (int64_t)counter.QuadPart;
#elif defined __linux || defined __linux__
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (int64_t)tp.tv_sec*1000000000 + tp.tv_nsec;
#elif defined __MACH__ && defined __APPLE__
    return (int64_t)mach_absolute_time();
#else
    struct timeval tv;
    struct timezone tz;
    gettimeofday( &tv, &tz );
    return (int64_t)tv.tv_sec*1000000 + tv.tv_usec;
#endif
  }

  double getTickFrequency(void)
  {
#if defined WIN32 || defined _WIN32 || defined WINCE
    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);
    return (double)freq.QuadPart;
#elif defined __linux || defined __linux__
    return 1e9;
#elif defined __MACH__ && defined __APPLE__
    static double freq = 0;
    if( freq == 0 )
    {
      mach_timebase_info_data_t sTimebaseInfo;
      mach_timebase_info(&sTimebaseInfo);
      freq = sTimebaseInfo.denom*1e9/sTimebaseInfo.numer;
    }
    return freq;
#else
    return 1e6;
#endif
  }

#if defined __GNUC__ && (defined __i386__ || defined __x86_64__ || defined __ppc__)
  #if defined(__i386__)
    int64_t getCPUTickCount(void)
    {
      int64 x;
      __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
      return x;
    }
  #elif defined(__x86_64__)
    int64_t getCPUTickCount(void)
    {
      unsigned hi, lo;
      __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
      return (int64_t)lo | ((int64_t)hi << 32);
    }
  #elif defined(__ppc__)
    int64_t getCPUTickCount(void)
    {
      int64 result = 0;
      unsigned upper, lower, tmp;
      __asm__ volatile(
                     "0:                  \n"
                     "\tmftbu   %0           \n"
                     "\tmftb    %1           \n"
                     "\tmftbu   %2           \n"
                     "\tcmpw    %2,%0        \n"
                     "\tbne     0b         \n"
                     : "=r"(upper),"=r"(lower),"=r"(tmp)
                     );
      return lower | ((int64)upper << 32);
    }
  #else
    #error "RDTSC not defined"
  #endif

#elif defined _MSC_VER && defined WIN32 && defined _M_IX86
  int64_t getCPUTickCount(void)
  {
    __asm _emit 0x0f;
    __asm _emit 0x31;
  }
#else
  int64_t getCPUTickCount(void)
  {
    return getTickCount();
  }
#endif

} /*  anonymous namespace */


