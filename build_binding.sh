rm -rf build/bindings/ompl
cmake -S . -B build
cmake --build build  -j 8