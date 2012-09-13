################################################################################################
# Finds Boost
macro(pcl_find_boost)
  if(NOT PCL_SHARED_LIBS OR WIN32)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC ON)
  endif(NOT PCL_SHARED_LIBS OR WIN32)

  if(${CMAKE_VERSION} VERSION_LESS 2.8.5)
    set(Boost_ADDITIONAL_VERSIONS "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46.1" "1.46.0" "1.46" "1.47" "1.47.0")
  else(${CMAKE_VERSION} VERSION_LESS 2.8.5)
    set(Boost_ADDITIONAL_VERSIONS "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0")
  endif(${CMAKE_VERSION} VERSION_LESS 2.8.5)

  # Disable the config mode of find_package(Boost)
  set(Boost_NO_BOOST_CMAKE ON)

  # Optional boost modules
  find_package(Boost 1.40.0 QUIET COMPONENTS serialization mpi)

  if(Boost_MPI_FOUND)
    set(BOOST_MPI_FOUND TRUE)
  endif(Boost_MPI_FOUND)

  if(Boost_SERIALIZATION_FOUND)
    set(BOOST_SERIALIZATION_FOUND TRUE)
  endif(Boost_SERIALIZATION_FOUND)

  # Required boost modules
  find_package(Boost 1.40.0 REQUIRED COMPONENTS system filesystem thread date_time iostreams)

  if(Boost_FOUND)  
    set(HAVE_BOOST ON)

    # Obtain diagnostic information about Boost's automatic linking outputted during compilation time.
    add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
    include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
  
    message(STATUS "Boost found (include: ${Boost_INCLUDE_DIRS})")
  endif()
endmacro()

################################################################################################
# Finds Eigen
macro(pcl_find_eigen)
  find_package(Eigen REQUIRED)
  include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
  add_definitions(-DEIGEN_USE_NEW_STDVECTOR -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

  if (EIGEN_FOUND)
    set(HAVE_EIGEN ON)
    pcl_parse_header("${EIGEN_INCLUDE_DIRS}/Eigen/src/Core/util/Macros.h" EIGEN_VERSION_LINES EIGEN_WORLD_VERSION EIGEN_MAJOR_VERSION EIGEN_MINOR_VERSION)   
  endif()
endmacro()


################################################################################################
# Finds FLANN
macro(pcl_find_flann)
  if(NOT PCL_SHARED_LIBS OR WIN32)
    set(FLANN_USE_STATIC ON)
  endif(NOT PCL_SHARED_LIBS OR WIN32)

  find_package(FLANN 1.7.0 REQUIRED)

  if (FLANN_FOUND)
    set(HAVE_FLANN ON)
    include_directories(SYSTEM ${FLANN_INCLUDE_DIRS})  
    pcl_parse_header2(FLANN "${FLANN_INCLUDE_DIRS}/flann/config.h" FLANN_VERSION_)   
  endif()
endmacro()

################################################################################################
# Finds VTK
macro(pcl_find_vtk)
 if (WITH_VTK)
    find_package(VTK)
  
    if (VTK_FOUND)        
      if (PCL_SHARED_LIBS OR (NOT (PCL_SHARED_LIBS) AND NOT (VTK_BUILD_SHARED_LIBS)))           
        link_directories(${VTK_LIBRARY_DIRS})
        
        set(HAVE_VTK ON)    
        message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
        pcl_parse_header("${VTK_INCLUDE_DIRS}/vtk-5.8/vtkConfigure.h" VTK_VERSION_LINES VTK_MAJOR_VERSION VTK_MINOR_VERSION VTK_BUILD_VERSION)
           
        find_package (QVTK)
      else ()    
        set(HAVE_VTK OFF)
        message ("Warning: You are to build PCL in STATIC but VTK is SHARED!")
        message ("Warning: VTK disabled!")
      endif ()   
    endif(VTK_FOUND)  
  endif()
endmacro()


################################################################################################
# Finds Qhull
macro(pcl_find_qhull)
  if(NOT PCL_SHARED_LIBS OR WIN32)
    set(QHULL_USE_STATIC ON)
  endif()
  
  find_package(Qhull)
  
  if (QHULL_FOUND)
    set(HAVE_QHULL ON)  
    include_directories(${QHULL_INCLUDE_DIRS})
    pcl_parse_header("${OPENNI_INCLUDE_DIRS}/XnVersion.h" OPENNI_VERSION_LINES XN_MAJOR_VERSION XN_MINOR_VERSION XN_MAINTENANCE_VERSION)  
  endif()
endmacro()

################################################################################################
# Finds OpenNI
macro(pcl_find_openni)
  find_package(OpenNI)
  
  if(OPENNI_FOUND)
    set(HAVE_OPENNI ON)    
    include_directories(SYSTEM ${OPENNI_INCLUDE_DIRS})   
    pcl_parse_header("${OPENNI_INCLUDE_DIRS}/XnVersion.h" XN_MAJOR_VERSION XN_MINOR_VERSION XN_MAINTENANCE_VERSION)  
  endif() 
endmacro()

################################################################################################
# Finds OpenMP
macro(pcl_find_openmp)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (HAVE_OPENMP 1)
    
    if(MSVC90 OR MSVC10)
      if(MSVC90)
        set(OPENMP_DLL VCOMP90)
      elseif(MSVC10)
        set(OPENMP_DLL VCOMP100)
      endif(MSVC90)
      set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
      set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
    endif(MSVC90 OR MSVC10)
  else()
    message (STATUS "OpenMP not found ")
  endif()
endmacro()

################################################################################################
# Finds all external libraries
# Usage: find_external_dependencies()
macro(find_external_dependencies)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

  # ---[ PCL1.x temporary ]---
  find_package ( PCL )
  if (PCL_FOUND)
    include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})   
    set(HAVE_PCL1x 1)
  endif()

  # ---[ Boost ]---
  pcl_find_boost()
  
  # ---[ Cuda ]---
  include(cmake/pcl_find_cuda.cmake)

  # ---[ Eigen ]--- 
  pcl_find_eigen()
   
  # ---[ FLANN ]---
  #pcl_find_flann()

  # ---[ OpenNI ]---
  if (WITH_OpenNI)
    pcl_find_openni()
  endif()
 
  # ---[ QHull ]---
  pcl_find_qhull()
   
  # ---[ VTK ]---
  pcl_find_vtk()

  # ---[ OpenMP ]---  
  pcl_find_openmp()
    
  # ---[ TBB ]---
  include(cmake/pcl_find_tbb.cmake)
  
  # ---[ MPI ]---
  if (WITH_MPI)
    find_package(MPI)
    if(MPI_CXX_FOUND)
      include_directories(SYSTEM ${MPI_INCLUDE_PATH})
    endif(MPI_CXX_FOUND)
  endif()

  # ---[ Doxygen and html help compiler ]---
  if (BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)  
      find_package(HTMLHelp)
    endif(DOXYGEN_FOUND)
  endif()

endmacro()


