FROM ubuntu:noble AS builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-program-options-dev \
    libboost-serialization-dev \
    libboost-test-dev \
    libeigen3-dev \
    libexpat1 \
    libflann-dev \
    libtriangle-dev \
    libyaml-cpp-dev \
    ninja-build \
    pkg-config \
    python3-dev \
    python3-pip \
    wget
COPY . /ompl
WORKDIR /ompl
RUN git submodule update --init --recursive && \
    cmake \
    -G Ninja \
    -B build \
    -DVAMP_PORTABLE_BUILD=ON \
    -DCMAKE_INSTALL_PREFIX=/usr && \
    cmake --build build && \
    cmake --install build && \
    cd tests/cmake_export && \
    cmake -B build -DCMAKE_INSTALL_PREFIX=../../install && \
    cmake --build build

FROM ubuntu:noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    libboost-program-options-dev \
    libboost-serialization-dev \
    libeigen3-dev \
    libflann-dev \
    libtriangle-dev \
    libyaml-cpp-dev \
    ninja-build \
    pkg-config \
    python3-dev \
    python3-pip \
    wget

COPY --from=builder /usr /usr
RUN useradd -ms /bin/bash ompl
USER ompl
WORKDIR /home/ompl
