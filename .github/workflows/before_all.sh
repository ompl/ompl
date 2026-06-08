#!/usr/bin/env bash

set -eux

build_os="$(uname)"
arch="$(uname -m)"

if [ "${build_os}" == "Linux" ]; then
    # Install base dependencies
    yum -y install \
        sudo

    if [ "${arch}" == "aarch64" ]; then
        yum install -y wget
        EIGEN_VERSION=3.4.0
        wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
        tar -xzf eigen-${EIGEN_VERSION}.tar.gz
        cd eigen-${EIGEN_VERSION}
        mkdir build && cd build
        cmake ..
        make install
    else
        yum -y install \
            eigen3-devel
    fi

elif [ "${build_os}" == "Darwin" ]; then
    echo "Detected macOS. Installing dependencies..."
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install \
        eigen \
        llvm@18 \
        yaml-cpp
fi
