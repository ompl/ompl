#!/usr/bin/env bash

set -eux

build_os="$(uname)"
arch="$(uname -m)"

if [ "${build_os}" == "Linux" ]; then
    # Install base dependencies
    yum -y install \
        sudo \
        eigen3-devel \
        llvm-devel-18.* \
        clang-devel-18.*

    if [ "${arch}" == "aarch64" ]; then
        echo "Detected AlmaLinux 8 AArch64. Applying architecture-specific fixes..."
        
        # Install additional dependencies for AArch64
        yum -y install \
            gcc gcc-c++ \
            libstdc++-devel \
            boost-python3-devel

        # Ensure we're using g++ instead of clang
        export CC=aarch64-linux-gnu-gcc 
        export CXX=aarch64-linux-gnu-g++
        # Ensure Boost-Python is found
        python_version=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
        echo "Python version: $python_version"
        boost_python_lib="/usr/local/lib/libboost_python${python_version//./}.so"
        
        if [ -f "$boost_python_lib" ]; then
            echo "Using Boost Python library: $boost_python_lib"
        else
            echo "Warning: Boost-Python library not found! Expected: $boost_python_lib"
        fi

        # Ensure PyPy is available
        if [ ! -L "/usr/bin/pypy" ] && [ -f "/opt/python/pp310-pypy310_pp73/bin/pypy" ]; then
            ln -s /opt/python/pp310-pypy310_pp73/bin/pypy /usr/bin
        else
            echo "Warning: PyPy not found or already symlinked."
        fi
        echo "Running make update_bindings to generate Python bindings..."
        make update_bindings VERBOSE=1 || echo "Warning: make update_bindings failed, check logs!"

    else
        # If not AArch64, still ensure PyPy is available
        if [ ! -L "/usr/bin/pypy" ] && [ -f "/opt/python/pp310-pypy310_pp73/bin/pypy" ]; then
            ln -s /opt/python/pp310-pypy310_pp73/bin/pypy /usr/bin
        fi
    fi

elif [ "${build_os}" == "Darwin" ]; then
    echo "Detected macOS. Installing dependencies..."
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install \
        eigen \
        pypy3 \
        castxml \
        llvm@18 \
        yaml-cpp
fi
