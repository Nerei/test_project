################################################################################################
# creates directory structure for new module
# Usage: pcl_create_module_template(module_name)
function(pcl_create_module_template)
  if (${ARGC} EQUAL "1")
    set(module_name ${ARGV0})
    set(cmake_file "CMakeFile2.txt")
    file(MAKE_DIRECTORY src)
    file(MAKE_DIRECTORY include/pcl/${module_name})

    file(WRITE  ${cmake_file} "pcl_define_module(${module_name} DOC \"PCL3 ${module_name} module\")\n")
    #file(APPEND ${cmake_file} "#end-of-file\n")
  endif()
endfunction()