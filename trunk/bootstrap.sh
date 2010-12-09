#!/usr/bin/env sh

BUILD_TYPE="RelWithDebInfo"
PYBINDINGS=1
JOBS=""

if [ "$1" = "--help" ]; then
    echo "Usage: ./bootstrap.sh [options]"
    echo "       -d, --debug                Build with debug symbols"
    echo "       -r, --release              Build with optimizations"
    echo "       --without-py-bindings      Do not build Python bindings"
    echo "       --jobs[=N]                 Jobs count to pass to 'make'"
    exit
fi

for arg in "$@"
do
    if [ "$arg" = "--release" ] || [ "$arg" = "-r" ]; then
	BUILD_TYPE="Release"
    fi
    if [ "$arg" = "--debug" ] || [ "$arg" = "-d" ]; then
	BUILD_TYPE="Debug"
    fi
    if [ "$arg" = "--without-py-bindings" ]; then
	PYBINDINGS=0
    fi
    if [ "${arg%=*}" = "--jobs" ]; then
	JOBS="$arg"
    fi
    
done

echo "Building OMPL ($BUILD_TYPE mode)..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE

if [ $PYBINDINGS -gt 0 ]; then
    make update_bindings
fi

make $JOBS
echo "Build complete."
echo "You can now run 'make test' in build/ and run demos from build/bin/"
cd ..
