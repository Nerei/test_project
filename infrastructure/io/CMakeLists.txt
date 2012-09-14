set(module_path "")
set(module_deps "")
set(module_opts "")

include_directories(${OPENNI_INCLUDE_DIRS})

add_definitions(-DPCLAPI_EXPORTS)

pcl_define_module(io DOC "pcl2 io module" DEPS core)

include_directories(../../modules/core/include)
target_link_libraries(pcl_io ${OPENNI_LIBRARIES})

pcl_set_default_source_groups()


