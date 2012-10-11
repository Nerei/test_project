if (NOT WITH_CUDA)
  return()
endif()

find_package(CUDA 4.1)

if(CUDA_FOUND)
  set(HAVE_CUDA ON)
  
  # Find a complete list for CUDA compute capabilities at http://developer.nvidia.com/cuda-gpus
    
  # CUDA_ARCH_BIN is a space separated list of GPU archs, for which binary GPU code should be embedded in output file. 
  # For example, CUDA_ARCH_BIN = "1.0 1.1 1.2 1.3 2.0". CUDA_ARCH_PTX is the same, but signals to include intermediate (PTX) code, 
  # which could be compiled to any newer or future architeture. For example, CUDA_ARCH_BIN = 1.3 can be compiled to 1.3, 2.0, 2.1, etc.   
  # Also it's possibe to specify virtual arch in parenthesis in order to limit instructions  set. 
  # To force compiler use only sm_11 instructions, se CUDA_ARCH_BIN = 11(11) 12(11) 13(11) 20(11) 21(11).
  # The CMake script interpret XX as XX (XX). This allows user to omit parenthesis for convinience.
  # An exceptional case is 2.1 arch, which doesn't have own sm_21 instructions set. For the achiteture option 21 = 21(21) is wrong, 
  # So need to manually force previous sm_20 instruction set via 21(20).
     
  if(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
      set(CUDA_ARCH_BIN "2.0 2.1(2.0) 3.0" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
  else()
      set(CUDA_ARCH_BIN "2.0 2.1(2.0)" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
  endif()

  set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")
  #set(CUDA_ARCH_PTX "1.1 1.2" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

  
  # Parse the CUDA_ARCH_BIN and CUDA_ARCH_PTR
  set(cuda_nvcc_target_flags "")
  string(REGEX REPLACE "\\." "" ARCH_BIN_WITHOUT_DOTS "${CUDA_ARCH_BIN}")
  string(REGEX REPLACE "\\." "" ARCH_PTX_WITHOUT_DOTS "${CUDA_ARCH_PTX}")

  set(cuda_nvcc_target_flags "")
  set(PCL_CUDA_ARCH_BIN "")
  set(PCL_CUDA_ARCH_PTX "")

  # Tell NVCC to add binaries for the specified GPUs
  string(REGEX MATCHALL "[0-9()]+" ARCH_LIST "${ARCH_BIN_WITHOUT_DOTS}")
  foreach(ARCH IN LISTS ARCH_LIST)
    if (ARCH MATCHES "([0-9]+)\\(([0-9]+)\\)")
      # User explicitly specified PTX for the concrete BIN
      set(cuda_nvcc_target_flags ${cuda_nvcc_target_flags} -gencode arch=compute_${CMAKE_MATCH_2},code=sm_${CMAKE_MATCH_1})
      set(PCL_CUDA_ARCH_BIN "${PCL_CUDA_ARCH_BIN} ${CMAKE_MATCH_1}")      
    else()
      # User didn't explicitly specify PTX for the concrete BIN, we assume PTX=BIN
      set(cuda_nvcc_target_flags ${cuda_nvcc_target_flags} -gencode arch=compute_${ARCH},code=sm_${ARCH})
      set(PCL_CUDA_ARCH_BIN "${PCL_CUDA_ARCH_BIN} ${ARCH}")      
    endif()
  endforeach()

  # Tell NVCC to add PTX intermediate code for the specified architectures
  string(REGEX MATCHALL "[0-9]+" ARCH_LIST "${ARCH_PTX_WITHOUT_DOTS}")
  foreach(ARCH IN LISTS ARCH_LIST)
    set(cuda_nvcc_target_flags ${cuda_nvcc_target_flags} -gencode arch=compute_${ARCH},code=compute_${ARCH})
    set(PCL_CUDA_ARCH_PTX "${PCL_CUDA_ARCH_PTX} ${ARCH}")
  endforeach()

  if (cuda_nvcc_target_flags)
    #message(STATUS "CUDA NVCC target flags: ${cuda_nvcc_target_flags}")
    list(APPEND CUDA_NVCC_FLAGS ${cuda_nvcc_target_flags})
  endif()

  mark_as_advanced(CUDA_BUILD_CUBIN CUDA_BUILD_EMULATION CUDA_VERBOSE_BUILD CUDA_SDK_ROOT_DIR)  
endif()
