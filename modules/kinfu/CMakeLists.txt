set(module_path "")
set(module_deps)
set(module_opts)

pcl_define_module(common DOC "PCL20 core module")

pcl_source_group("Header Files\\clouds" GLOB "include/pcl/${module_name}/clouds/*.h*" "include/pcl/${module_name}/cuda/*.h*")
pcl_source_group("Source Files\\clouds" GLOB "src/clouds/*.cpp")

pcl_add_module_tests()



add_subdirectory(app)