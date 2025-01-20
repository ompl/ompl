#!/usr/bin/env bash

set -eux

build_os="$(uname)"
build_arch="$(uname -m)"

if [ "${build_os}" == "Linux" ]; then
    if [ "${build_arch}" == "aarch64" ] || [ "${build_arch}" == "arm64" ]; then
        # Ubuntu ARM specific installations
        apt-get update
        apt-get install -y \
            sudo \
            libeigen3-dev \
            llvm-dev \
            clang \
            python3-pip \
            python3-venv
  
        if ! command -v pypy3 &> /dev/null; then
            sudo add-apt-repository ppa:deadsnakes/ppa
            sudo apt-get update
            sudo apt-get install -y pypy3
        fi
    else
        # Assuming other Linux architectures are Red Hat based and use yum
        yum -y install \
            sudo \
            eigen3 \
            llvm-devel \
            clang-devel
    
        # manylinux ships with a pypy installation. Make it available on the $PATH
        # so the OMPL build process picks it up and can make use of it during the
        # Python binding generation stage.
        ln -s /opt/python/pp310-pypy310_pp73/bin/pypy /usr/bin/pypy
    fi
elif [ "${build_os}" == "Darwin" ]; then
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install \
        eigen \
        pypy3 \
        castxml \
        llvm@18
fi
