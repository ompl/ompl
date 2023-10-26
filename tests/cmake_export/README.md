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