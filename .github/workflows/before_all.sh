#!/usr/bin/env bash

set -eux

build_os="$(uname)"
arch="$(uname -m)"

if [ "${build_os}" == "Linux" ]; then
    # Install base dependencies
    yum -y install \
        sudo \
        eigen3-devel \
        llvm-devel \
        clang-devel \
        boost-devel

    if [ "${arch}" == "aarch64" ]; then
        echo "Detected AlmaLinux 8 AArch64. Applying architecture-specific fixes..."
        
        # Install additional dependencies for AArch64
        yum -y install \
            gcc gcc-c++ \
            libstdc++-devel
        # Ensure we're using g++ instead of clang
        export CC=aarch64-linux-gnu-gcc 
        export CXX=aarch64-linux-gnu-g++
        # Ensure Boost-Python is found
        python_version=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
        echo "Python version: $python_version"
    fi
elif [ "${build_os}" == "Darwin" ]; then
    echo "Detected macOS. Installing dependencies..."
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install \
        eigen \
        llvm@18
fi
