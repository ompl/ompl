This folder is for testing the CMake export of ompl is working. 

```bash
cd ompl
# First, try default options.
# For maximium robustness, set up a matrix of options to check.
cmake -B build
cmake --build build
cmake --install build --prefix install
cd tests/cmake_export
cmake -B build -DCMAKE_INSTALL_PREFIX=../../install
cmake --build build
```


add_subdirectory(ompl)
target_include_directories(ompl
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    $<${BUILD_INTERFACE}:${CMAKE_INSTALL_INCLUDEDIR}/foobar)
  # The install include directory will be set during install().
-- Configuring done
gmake: *** [Makefile:948: cmake_check_build_system] Segmentation fault (core dumped)