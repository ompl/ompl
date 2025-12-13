file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.1/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/CMakeModules/CPM.cmake
  EXPECTED_HASH SHA256=117cbf2711572f113bab262933eb5187b08cfc06dce0714a1ee94f2183ddc3ec
)
set(CPM_USE_LOCAL_PACKAGES ON)
include(${CMAKE_CURRENT_BINARY_DIR}/CMakeModules/CPM.cmake)
