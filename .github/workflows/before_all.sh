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
        yum -y install gcc gcc-c++ libstdc++-devel
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
