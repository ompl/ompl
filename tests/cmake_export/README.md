This folder is for testing the CMake export of ompl is working. TODO add this to CI. 

```bash
cd ompl
cmake -B build
cmake --build build
cmake --install build --prefix install
cd tests/cmake_export
cmake -B build -DCMAKE_INSTALL_PREFIX=../../install
cmake --build build
```