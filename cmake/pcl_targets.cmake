################################################################################################
# Short command for picking up source in default paths
# Usage: pcl_pickup_sources()
macro(pcl_pickup_sources)
  FILE(GLOB_RECURSE sources include/pcl2/${module_name}/*.h* src/*.cpp src/*.h*)
endmacro()


################################################################################################
# Add PCL module libarary
# Usage: pcl_add_library()
function(pcl_add_library)
  set(__name pcl_${module_name})
  add_library(${__name} SHARED ${sources})

  # Only link if needed
  if(MSVC)
    set_target_properties(${__name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set_target_properties(${__name} PROPERTIES LINK_FLAGS -Wl)
  elseif(__COMPILER_PATHSCALE)
    set_target_properties(${__name} PROPERTIES LINK_FLAGS -mp)
  else()
    set_target_properties(${__name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
  endif()

  set_target_properties(${__name} PROPERTIES
    VERSION ${PCL_VERSION}
    SOVERSION ${PCL_MAJOR_VERSION}.${PCL_MINOR_VERSION}
    DEFINE_SYMBOL "PCLAPI_EXPORTS")

  if(USE_PROJECT_FOLDERS)
    set_target_properties(${__name} PROPERTIES FOLDER "Libraries")
  endif()
endfunction()

################################################################################################
# Usage: pcl_setup_includes()
macro(pcl_setup_includes)
    include_directories(include)
endmacro()

################################################################################################
# Function for adding precompiled headers
# Usage: pcl_add_precompiled_headers()
function(pcl_add_precompiled_headers)
  message(STATUS "Precompiled headers are not implemented in CMake scripts")
  return()
  set(pch_header "${CMAKE_CURRENT_SOURCE_DIR}/src/precomp.hpp")

  if(PCHSupport_FOUND AND ENABLE_PRECOMPILED_HEADERS AND EXISTS "${pch_header}")
    if(CMAKE_GENERATOR MATCHES Visual)
      set(${module_name}_pch "${CMAKE_CURRENT_SOURCE_DIR}/src/precomp.cpp")
      add_native_precompiled_header(${module_name} ${pch_header})
    elseif(CMAKE_GENERATOR MATCHES Xcode)
      add_native_precompiled_header(${module_name} ${pch_header})
    elseif(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_GENERATOR MATCHES "Makefiles|Ninja")
      add_precompiled_header(${module_name} ${pch_header})
    endif()
  endif()
endfunction()


################################################################################################
# Short command for adding a test target.
# Usage: pcl_add_test(name FILES files ARGUMENTS command line arguments LINK_WITH link libraries)
function(pcl_add_test name)
  set(multiValueArgs FILES ARGUMENTS LINK_WITH)
  cmake_parse_arguments(PCL_ADD_TEST "" "" "${multiValueArgs}" ${ARGN})

  add_executable(${name} ${PCL_ADD_TEST_FILES})
  target_link_libraries(${name} ${PCL_ADD_TEST_LINK_WITH})

  if(MSVC)
    set_target_properties(${_exename} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set_target_properties(${_exename} PROPERTIES LINK_FLAGS -Wl)
    target_link_libraries(${_exename} pthread)
  elseif(UNIX AND NOT ANDROID_NDK)
    set_target_properties(${_exename} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    # GTest >= 1.5 requires pthread and CMake's 2.8.4 FindGTest is broken
    #target_link_libraries(${_exename})
  endif()

  if(USE_PROJECT_FOLDERS)
    set_target_properties(${name} PROPERTIES FOLDER "Tests")
  endif()

  if(${CMAKE_VERSION} VERSION_LESS 2.8.4)
    add_test(${name} ${name} ${PCL_ADD_TEST_ARGUMENTS})
  else()
    add_test(NAME ${name} COMMAND ${name} ${PCL_ADD_TEST_ARGUMENTS})
  endif()
endfunction()


################################################################################################
# short command for declaring pcl module
# Usage: pcl_define_module(module_name [DOC Description] [DEPS dependecies] [OPTIONAL optional_dependencies])
macro(pcl_define_module name)
  set(multiValueArgs DEPS OPTIONAL)
  cmake_parse_arguments(PCL_DEFINE_MODULE "" "DOC" "${multiValueArgs}" ${ARGN})
  cmake_parse_arguments(PCL_MODULE_DEPS_VAR "" "DOC" "${multiValueArgs}" ${module_deps})

  set(module_name ${name})
  set(module_desc ${PCL_DEFINE_MODULE_DOC})
  set(module_deps "")

  list(APPEND module_deps ${PCL_MODULE_DEPS_VAR_UNPARSED_ARGUMENTS} ${PCL_DEFINE_MODULE_DEPS} OPTIONAL ${PCL_MODULE_DEPS_VAR_OPTIONAL} ${PCL_DEFINE_MODULE_OPTIONAL} )
  list(REMOVE_DUPLICATES module_deps)

  if (NOT DEFINED module_path)
    set(module_path "")
  endif()

  pcl_pickup_sources()
  pcl_setup_includes()
  pcl_add_library()

#  pcl_add_precompiled_headers()

endmacro()

################################################################################################
# Short command for adding default tests
# Usage: pcl_add_module_tests()
function(pcl_add_module_tests)
  set(__test_path ${CMAKE_CURRENT_SOURCE_DIR}/test)
  if(BUILD_TESTS AND EXISTS "${__test_path}")
    file(GLOB __srcs "${__test_path}/*.cpp" "${__test_path}/*.h*")
    pcl_add_test(test_${module_name} FILES ${__srcs} LINK_WITH pcl_${module_name})
  endif()
  unset(__test_path)
endfunction()

