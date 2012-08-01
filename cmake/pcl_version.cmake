################################################################################################
# Declares PCL version and its components
# Usage: declare_pcl_version(version)

macro(declare_pcl_version version)
  set(PCL_VERSION ${version} CACHE STRING "PCL version")

  # Find version components
  string(REGEX REPLACE "^([0-9]+).*" "\\1"                    PCL_MAJOR_VERSION     "${PCL_VERSION}")
  string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"           PCL_MINOR_VERSION     "${PCL_VERSION}")
  string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1"    PCL_REVISION_VERSION  "${PCL_VERSION}")
  string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1"  PCL_CANDIDATE_VERSION "${PCL_VERSION}")
endmacro()

if (0) 
  # this is bad!!!  Use version header instead!
  # because users not always will parse PCLConfigVersion.cmake version
  declare_pcl_version(2.0.0)
else()
  set(PCL_VERSION_FILE "${CMAKE_SOURCE_DIR}/modules/core/include/pcl2/core/version.hpp")
  
  pcl_parse_header("${PCL_VERSION_FILE}" PCL_VERSION_LINES PCL_MAJOR_VERSION PCL_MINOR_VERSION PCL_REVISION_VERSION PCL_CANDIDATE_VERSION)
  set(PCL_VERSION "${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}.${PCL_REVISION_VERSION}")
 
  #Ceate a dependency on version file. We never use output of the following command but cmake will rerun automatically if the version file changes
  configure_file("${PCL_VERSION_FILE}" "${CMAKE_BINARY_DIR}/junk/version.junk" COPYONLY)
endif()







