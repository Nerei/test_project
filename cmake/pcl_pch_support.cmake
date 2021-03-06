# taken from http://public.kitware.com/Bug/view.php?id=1260 and slightly adjusted
# - Try to find precompiled headers support for GCC 3.4 and 4.x
# Once done this will define:
#
# Variable:
#   PCHSupport_FOUND
#
# Macro:
#   ADD_PRECOMPILED_HEADER  _targetName _input  _dowarn
#   ADD_PRECOMPILED_HEADER_TO_TARGET _targetName _input _pch_output_to_use _dowarn
#   ADD_NATIVE_PRECOMPILED_HEADER _targetName _input _dowarn
#   GET_NATIVE_PRECOMPILED_HEADER _targetName _input

if(CMAKE_COMPILER_IS_GNUCXX)
  exec_program(${CMAKE_CXX_COMPILER}
               ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
               OUTPUT_VARIABLE gcc_compiler_version)

  if(gcc_compiler_version MATCHES "4\\.[0,2-9]\\.[0-9]")
    set(PCHSupport_FOUND TRUE)
  endif()

  set(_PCH_include_prefix "-I")
  set(_PCH_isystem_prefix "-isystem")

elseif(CMAKE_GENERATOR MATCHES "^Visual.*$")

  set(PCHSupport_FOUND TRUE)
  set(_PCH_include_prefix "/I")
  set(_PCH_isystem_prefix "/I")

else()
  set(PCHSupport_FOUND FALSE)
endif()

macro(_PCH_GET_COMPILE_FLAGS _out_compile_flags)
  string(TOUPPER "CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}" _flags_var_name)
  set(${_out_compile_flags} ${${_flags_var_name}} )

  if(CMAKE_COMPILER_IS_GNUCXX)

    get_target_property(_targetType ${_PCH_current_target} TYPE)
    if(${_targetType} STREQUAL SHARED_LIBRARY AND NOT WIN32)
      LIST(APPEND ${_out_compile_flags} "-fPIC")
    endif()

  else()
    ## TODO ... ? or does it work out of the box
  endif()

  get_directory_property(DIRINC INCLUDE_DIRECTORIES )
  foreach(item ${DIRINC})
    if(item MATCHES "^${OpenCV_SOURCE_DIR}/modules/")
      list(APPEND ${_out_compile_flags} "${_PCH_include_prefix}\"${item}\"")
    else()
      list(APPEND ${_out_compile_flags} "${_PCH_isystem_prefix}\"${item}\"")
    endif()
  endforeach(item)

  get_directory_property(_directory_flags DEFINITIONS)
  get_directory_property(_global_definitions DIRECTORY ${OpenCV_SOURCE_DIR} DEFINITIONS)
  #MESSAGE("_directory_flags ${_directory_flags} ${_global_definitions}" )
  list(APPEND ${_out_compile_flags} ${_directory_flags})
  list(APPEND ${_out_compile_flags} ${_global_definitions})
  list(APPEND ${_out_compile_flags} ${CMAKE_CXX_FLAGS})

  separate_arguments(${_out_compile_flags})
endmacro(_PCH_GET_COMPILE_FLAGS)

macro(_PCH_WRITE_PCHDEP_CXX _targetName _include_file _dephelp)
  set(${_dephelp} ${CMAKE_CURRENT_BINARY_DIR}/${_targetName}_pch_dephelp.cxx)
  if(CMAKE_HOST_WIN32)
    add_custom_command(
      OUTPUT "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "#include \\\"${_include_file}\\\"" >  "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "int testfunction();"               >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "int testfunction()"                >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "{"                                 >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "    return 0;"                     >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "}"                                 >> "${${_dephelp}}"
      DEPENDS "${_include_file}"
      )
  else()
    add_custom_command(
      OUTPUT "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "\\#include \\\"${_include_file}\\\"" >  "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "int testfunction\\(\\)\\;"         >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "int testfunction\\(\\)"            >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "{"                                 >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "    \\return 0\\;"                 >> "${${_dephelp}}"
      COMMAND ${CMAKE_COMMAND} -E echo "}"                                 >> "${${_dephelp}}"
      DEPENDS "${_include_file}"
      )
  endif()
endmacro(_PCH_WRITE_PCHDEP_CXX )

macro(_PCH_GET_COMPILE_COMMAND out_command _input _output)
  file(TO_NATIVE_PATH ${_input} _native_input)
  file(TO_NATIVE_PATH ${_output} _native_output)

  if(CMAKE_COMPILER_IS_GNUCXX)
    if(CMAKE_CXX_COMPILER_ARG1)
      # remove leading space in compiler argument
      string(REGEX REPLACE "^ +" "" pchsupport_compiler_cxx_arg1 ${CMAKE_CXX_COMPILER_ARG1})
      set(${out_command} ${CMAKE_CXX_COMPILER} ${pchsupport_compiler_cxx_arg1} ${_compile_FLAGS} -x c++-header -o ${_output} ${_input})

    else(CMAKE_CXX_COMPILER_ARG1)
      set(${out_command} ${CMAKE_CXX_COMPILER}  ${_compile_FLAGS} -x c++-header -o ${_output} ${_input})
    endif(CMAKE_CXX_COMPILER_ARG1)
  else(CMAKE_COMPILER_IS_GNUCXX)

    set(_dummy_str "#include <${_input}>")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/pch_dummy.cpp ${_dummy_str})
    set(${out_command} ${CMAKE_CXX_COMPILER} ${_compile_FLAGS} /c /Fp${_native_output} /Yc${_native_input} pch_dummy.cpp) #/out:${_output}

  endif(CMAKE_COMPILER_IS_GNUCXX)
endmacro(_PCH_GET_COMPILE_COMMAND )

macro(_PCH_GET_TARGET_COMPILE_FLAGS _cflags  _header_name _pch_path _dowarn )
  file(TO_NATIVE_PATH ${_pch_path} _native_pch_path)

  if(CMAKE_COMPILER_IS_GNUCXX)
    # for use with distcc and gcc >4.0.1 if preprocessed files are accessible
    # on all remote machines set
    # PCH_ADDITIONAL_COMPILER_FLAGS to -fpch-preprocess
    # if you want warnings for invalid header files (which is very inconvenient
    # if you have different versions of the headers for different build types
    # you may set _pch_dowarn
    if (_dowarn)
      set(${_cflags} "${PCH_ADDITIONAL_COMPILER_FLAGS} -include \"${CMAKE_CURRENT_BINARY_DIR}/${_header_name}\" -Winvalid-pch " )
    else (_dowarn)
      set(${_cflags} "${PCH_ADDITIONAL_COMPILER_FLAGS} -include \"${CMAKE_CURRENT_BINARY_DIR}/${_header_name}\" " )
    endif (_dowarn)

  else(CMAKE_COMPILER_IS_GNUCXX)
    set(${_cflags} "/Fp${_native_pch_path} /Yu${_header_name}" )
  endif(CMAKE_COMPILER_IS_GNUCXX)
endmacro(_PCH_GET_TARGET_COMPILE_FLAGS )


macro(GET_PRECOMPILED_HEADER_OUTPUT _targetName _input _output)
  get_filename_component(_name ${_input} NAME)
  get_filename_component(_path ${_input} PATH)
  set(${_output} "${CMAKE_CURRENT_BINARY_DIR}/${_name}.gch/${_targetName}_${CMAKE_BUILD_TYPE}.gch")
endmacro(GET_PRECOMPILED_HEADER_OUTPUT _targetName _input)

macro(ADD_PRECOMPILED_HEADER_TO_TARGET _targetName _input _pch_output_to_use )
  # to do: test whether compiler flags match between target  _targetName
  # and _pch_output_to_use
  get_filename_component(_name ${_input} NAME)

  if(ARGN STREQUAL "0")
    set(_dowarn 0)
  else()
    set(_dowarn 1)
  endif()

  _PCH_GET_TARGET_COMPILE_FLAGS(_target_cflags ${_name} ${_pch_output_to_use} ${_dowarn})
  #MESSAGE("Add flags ${_target_cflags} to ${_targetName} " )
  set_target_properties(${_targetName} PROPERTIES COMPILE_FLAGS ${_target_cflags} )

  add_custom_target(pch_Generate_${_targetName} DEPENDS ${_pch_output_to_use} )
  add_dependencies(${_targetName} pch_Generate_${_targetName} )
endmacro(ADD_PRECOMPILED_HEADER_TO_TARGET)

macro(ADD_PRECOMPILED_HEADER _targetName _input)

  set(_PCH_current_target ${_targetName})

  if(NOT CMAKE_BUILD_TYPE)
      message(FATAL_ERROR "This is the ADD_PRECOMPILED_HEADER macro. You must set CMAKE_BUILD_TYPE!")
  endif()

  if(ARGN STREQUAL "0")
      set(_dowarn 0)
  else()
      set(_dowarn 1)
  endif()

  get_filename_component(_name ${_input} NAME)
  get_filename_component(_path ${_input} PATH)
  GET_PRECOMPILED_HEADER_OUTPUT( ${_targetName} ${_input} _output)

  _PCH_WRITE_PCHDEP_CXX(${_targetName} "${_input}" _pch_dephelp_cxx)

  add_library(${_targetName}_pch_dephelp STATIC "${_pch_dephelp_cxx}" "${_input}" )

  set_target_properties(${_targetName}_pch_dephelp PROPERTIES
    DEBUG_POSTFIX "${OPENCV_DEBUG_POSTFIX}"
    ARCHIVE_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
    )

  _PCH_GET_COMPILE_FLAGS(_compile_FLAGS)

  #message("_compile_FLAGS: ${_compile_FLAGS}")
  #message("COMMAND ${CMAKE_CXX_COMPILER}    ${_compile_FLAGS} -x c++-header -o ${_output} ${_input}")

  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${_name}"
    COMMAND ${CMAKE_COMMAND} -E copy  "${_input}" "${CMAKE_CURRENT_BINARY_DIR}/${_name}" # ensure same directory! Required by gcc
    DEPENDS "${_input}"
    )

  #message("_command  ${_input} ${_output}")
  _PCH_GET_COMPILE_COMMAND(_command  ${CMAKE_CURRENT_BINARY_DIR}/${_name} ${_output} )

  get_filename_component(_outdir ${_output} PATH)
  add_custom_command(
    OUTPUT "${_output}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_outdir}"
    COMMAND ${_command}
    DEPENDS "${_input}"
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/${_name}"
    DEPENDS ${_targetName}_pch_dephelp
    )

  ADD_PRECOMPILED_HEADER_TO_TARGET(${_targetName} ${_input}  ${_output} ${_dowarn})
endmacro(ADD_PRECOMPILED_HEADER)


# Generates the use of precompiled in a target,
# without using depency targets (2 extra for each target)
# Using Visual, must also add ${_targetName}_pch to sources
# Not needed by Xcode

macro(GET_NATIVE_PRECOMPILED_HEADER _targetName _input)
  if(CMAKE_GENERATOR MATCHES "^Visual.*$")
    set(_dummy_str "#include \"${_input}\"\n"
"// This is required to suppress LNK4221.  Very annoying.\n"
"void *g_${_targetName}Dummy = 0\;\n")

    # Use of cxx extension for generated files (as Qt does)
    set(${_targetName}_pch ${CMAKE_CURRENT_BINARY_DIR}/${_targetName}_pch.cxx)
    if(EXISTS ${${_targetName}_pch})
      # Check if contents is the same, if not rewrite
      # todo
    else()
      file(WRITE ${${_targetName}_pch} ${_dummy_str})
    endif()
  endif()
endmacro(GET_NATIVE_PRECOMPILED_HEADER)


macro(ADD_NATIVE_PRECOMPILED_HEADER _targetName _input)
  if(ARGN STREQUAL "0")
    set(_dowarn 0)
  else()
    set(_dowarn 1)
  endif()

  if(CMAKE_GENERATOR MATCHES "^Visual.*$")

    # Auto include the precompile (useful for moc processing, since the use of
    # precompiled is specified at the target level
    # and I don't want to specifiy /F- for each moc/res/ui generated files (using Qt)

    get_target_property(oldProps ${_targetName} COMPILE_FLAGS)
    if (oldProps MATCHES NOTFOUND)
        set(oldProps "")
    endif()

    set(newProperties "${oldProps} /Yu\"${_input}\" /FI\"${_input}\"")
    set_target_properties(${_targetName} PROPERTIES COMPILE_FLAGS "${newProperties}")

    #also inlude ${oldProps} to have the same compile options
    set_source_files_properties(${${_targetName}_pch} PROPERTIES COMPILE_FLAGS "${oldProps} /Yc\"${_input}\"")

  elseif (CMAKE_GENERATOR MATCHES Xcode)

    # For Xcode, cmake needs my patch to process
    # GCC_PREFIX_HEADER and GCC_PRECOMPILE_PREFIX_HEADER as target properties

    # When buiding out of the tree, precompiled may not be located
    # Use full path instead.
    get_filename_component(fullPath ${_input} ABSOLUTE)

    set_target_properties(${_targetName} PROPERTIES XCODE_ATTRIBUTE_GCC_PREFIX_HEADER "${fullPath}")
    set_target_properties(${_targetName} PROPERTIES XCODE_ATTRIBUTE_GCC_PRECOMPILE_PREFIX_HEADER "YES")

  else()
    #Fallback to the "old" precompiled suppport
    #ADD_PRECOMPILED_HEADER(${_targetName} ${_input} ${_dowarn})
  endif()
endmacro(ADD_NATIVE_PRECOMPILED_HEADER)

macro(pcl_add_precompiled_header_to_target the_target pch_header)
  if(PCHSupport_FOUND AND ENABLE_PRECOMPILED_HEADERS AND EXISTS "${pch_header}")
    if(CMAKE_GENERATOR MATCHES Visual)
      string(REGEX REPLACE "hpp$" "cpp" ${the_target}_pch "${pch_header}")
      add_native_precompiled_header(${the_target} ${pch_header})
      unset(${the_target}_pch)
    elseif(CMAKE_GENERATOR MATCHES Xcode)
      add_native_precompiled_header(${the_target} ${pch_header})
    elseif(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_GENERATOR MATCHES "Makefiles|Ninja")
      add_precompiled_header(${the_target} ${pch_header})
    endif()
  endif()
endmacro()