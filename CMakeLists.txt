cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# ---[ Configurations types
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Possible configurations" FORCE)

# ---[ PCL 2 solution name
project(PCL2 C CXX)

# ---[ Required cmake scripts
include(cmake/pcl_utils.cmake)
include(cmake/pcl_version.cmake)
include(cmake/pcl_warnings.cmake)
include(cmake/pcl_template.cmake)
include(cmake/pcl_configs.cmake)
include(cmake/pcl_summary.cmake)
include(cmake/pcl_find_extdeps.cmake)
#include(cmake/pcl_pch_support.cmake)
include(cmake/pcl_targets.cmake)

# ---[ PCL 2.0 options
pcl_option(BUILD_SANDBOX              "Build sandbox modules"                             OFF)
pcl_option(BUILD_TESTS                "Build PCL tests"                                   ON)
pcl_option(BUILD_DOCS                 "Build documentation tests"                         ON)
pcl_option(WITH_CUDA                  "Build with CUDA support"                           ON)
pcl_option(WITH_TBB                   "Include Intel TBB support"                         OFF IF (NOT IOS) )
pcl_option(WITH_VTK                   "Build with Qhull support"                          ON)
pcl_option(WITH_OpenNI                "Build with OpenNI support"                         ON)
pcl_option(USE_PROJECT_FOLDERS        "Use folders to organize PCL projects in an IDE."   OFF)
pcl_option(ENABLE_NOISY_WARNINGS      "Show all warnings even if they are too noisy"      OFF)
pcl_option(ENABLE_PRECOMPILED_HEADERS "Use precompiled headers"                           ON  IF (NOT IOS) ) 

# ---[ Find external dependencies
find_external_dependencies()

# ---[ Misc settings
if(USE_PROJECT_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

# ---[ Disable noisy warnigns ( with compiler flag support check )

# pcl_warnings_disable(CMAKE_CXX_FLAGS -Wundef -Wmissing-declarations /wd4211 /wd4201 /wd4100 /wd4505 /wd4408)

generate_pcl_config_h_file()

# ---[ include modules into the project
add_subdirectory(modules)
add_subdirectory(infrastructure)
add_subdirectory(sandbox)



print_configuration_summary()

