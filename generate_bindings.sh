rm -rf build/bindings/ompl
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build  -j 6