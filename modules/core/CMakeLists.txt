set(module_path "")
set(module_deps "")
set(module_opts "")

include_directories(${CUDA_INCLUDE_DIRS})

add_definitions(-DPCLAPI_EXPORTS)

pcl_define_module(core DOC "pcl2 core module")


target_link_libraries(pcl_core ${CUDA_LIBRARIES})

pcl_set_default_source_groups()

pcl_source_group("Include\\clouds" GLOB "include/pcl2/${module_name}/clouds/*.h*")
pcl_source_group("Include\\cuda"   GLOB "include/pcl2/${module_name}/cuda/*.h*")
pcl_source_group("Source\\clouds"  GLOB "src/clouds/*.cpp")


#pcl_add_precompiled_header_to_target(pcl_${module_name} "${CMAKE_CURRENT_SOURCE_DIR}/src/precomp.hpp")


pcl_add_module_tests()

