FROM ubuntu:jammy AS builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        castxml \
        cmake \
        freeglut3-dev \
        git \
        libassimp-dev \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libccd-dev \
        libeigen3-dev \
        libexpat1 \
        libfcl-dev \
        libflann-dev \
        libode-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-celery \
        python3-dev \
        python3-flask \
        python3-numpy \
        python3-opengl \
        python3-pip \
        python3-pyqt5.qtopengl \
        pypy3 \
        wget && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    # see https://github.com/mcfletch/pyopengl/issues/74
    pip3 install pygccxml pyplusplus git+https://github.com/mcfletch/pyopengl.git@227f9c66976d9f5dadf62b9a97e6beaec84831ca#subdirectory=accelerate
COPY . /omplapp
WORKDIR /build
RUN cmake \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -G Ninja \
        /omplapp && \
    ninja update_bindings -j `nproc` && \
    ninja -j `nproc` && \
    ninja install

FROM ubuntu:jammy
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        clang \
        cmake \
        freeglut3-dev \
        git \
        libassimp-dev \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libccd-dev \
        libeigen3-dev \
        libfcl-dev \
        libflann-dev \
        libode-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python-celery-common \
        python3-celery \
        python3-dev \
        python3-flask \
        python3-numpy \
        python3-opengl \
        python3-pip \
        python3-pyqt5.qtopengl \
        uwsgi-plugin-python3 \
        wget && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    # install PyOpenGL
    # see https://github.com/mcfletch/pyopengl/issues/74
    pip install git+https://github.com/mcfletch/pyopengl.git@227f9c66976d9f5dadf62b9a97e6beaec84831ca#subdirectory=accelerate

COPY --from=builder /usr /usr
RUN useradd -ms /bin/bash ompl
USER ompl
WORKDIR /home/ompl
