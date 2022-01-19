include(CMakeFindDependencyMacro)

find_dependency(Eigen3)

if(@EbsdLib_ENABLE_HDF5@)
  find_dependency(H5Support)
endif()

if(@EbsdLib_USE_PARALLEL_ALGORITHMS@)
  find_dependency(TBB)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/EbsdLibTargets.cmake")

set(EbsdLib_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../include")
set(EbsdLib_LIB_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../lib;${CMAKE_CURRENT_LIST_DIR}/../../bin")
