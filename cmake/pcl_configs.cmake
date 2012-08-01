
################################################################################################
# Generated config.h file
# Usage: generate_pcl_config_h_file()
macro(generate_pcl_config_h_file)
  set(ifile "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/pcl_config.h.in")
  set(ofile "${CMAKE_CURRENT_BINARY_DIR}/include/pcl2/pcl_config.h")
  set(PCL_CONFIG_FILE_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/")
  
  configure_file(${ifile} ${ofile})
  include_directories(${PCL_CONFIG_FILE_INCLUDE_DIR})   
endmacro()
