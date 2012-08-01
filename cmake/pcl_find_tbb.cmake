if(NOT WITH_TBB)
  return()
endif()

if (ANDROID)
  add_subdirectory("${CMAKE_SOURCE_DIR}/3rdparty/tbb")

  set(HAVE_TBB ON)
  set(TBB_LIBRARIES tbb)

  add_definitions(-DTBB_USE_GCC_BUILTINS=1 -D__TBB_GCC_BUILTIN_ATOMICS_PRESENT=1)
  if(tbb_need_GENERIC_DWORD_LOAD_STORE)
    add_definitions(-D__TBB_USE_GENERIC_DWORD_LOAD_STORE=1)
  endif()

  elseif(UNIX AND NOT APPLE)
  find_package(PkgConfig QUIET)

  if(PKG_CONFIG_FOUND)
    pkg_check_modules(TBB tbb)
  endif(PKG_CONFIG_FOUND)

  if(TBB_FOUND)
    set(HAVE_TBB ON)
  endif()
endif()

if (NOT HAVE_TBB)
  find_path(TBB_INCLUDE_DIRS "tbb/tbb.h"
        PATHS
          "/usr/include"
          "/opt/intel/tbb"
          "/usr/local/include"
          "C:/Program Files/Intel/tbb/include"
          "C:/Program Files (x86)/tbb/include"
          "C:/Program Files (x86)/Intel/TBB"
          "${CMAKE_INSTALL_PREFIX}/include"
        DOC
          "The path to TBB headers")

  if(TBB_INCLUDE_DIRS)
    set(HAVE_TBB ON)

    get_filename_component(___tbb_libs_path "${TBB_INCLUDE_DIRS}/../lib" ABSOLUTE)

    if (MSVC)
      if(CMAKE_SYSTEM_PROCESSOR MATCHES amd64*|x86_64* OR MSVC64)
        set(___tbb_libs_path "${___tbb_libs_path}/intel64")
      else()
        set(___tbb_libs_path "${___tbb_libs_path}/ia32")
      endif()

      if(MSVC80)
        set(___tbb_libs_path "${___tbb_libs_path}/vc8")
      elseif(MSVC90)
        set(___tbb_libs_path "${___tbb_libs_path}/vc9")
      elseif(MSVC10)
        set(___tbb_libs_path "${___tbb_libs_path}/vc10")
      elseif(MSVC11)
        set(___tbb_libs_path "${___tbb_libs_path}/vc11")
      endif()
    endif()

    set(TBB_LIBRARY_DIRS "${___tbb_libs_path}" CACHE PATH "Full path of TBB library directory")
    unset(___tbb_libs_path)

    if (APPLE)
       set(TBB_LIBRARIES libtbb.dylib)
    elseif(UNIX OR (WIN32 AND CMAKE_COMPILER_IS_GNUCXX))
       set(TBB_LIBRARIES tbb)
    endif()

  endif()
endif()


if (HAVE_TBB)
  include_directories(SYSTEM ${TBB_INCLUDE_DIRS})
  link_directories(${TBB_LIBRARY_DIRS})
  pcl_parse_header("${TBB_INCLUDE_DIRS}/tbb/tbb_stddef.h" TBB_VERSION_LINES TBB_VERSION_MAJOR TBB_VERSION_MINOR TBB_INTERFACE_VERSION)
endif()