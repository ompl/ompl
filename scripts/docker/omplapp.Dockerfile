FROM ubuntu:bionic AS builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        build-essential  \
        cmake \
        freeglut3-dev \
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
        libtinfo5 \
        libtriangle-dev \
        pkg-config \
        python3-celery \
        python3-dev \
        python3-flask \
        python3-numpy \
        python3-opengl \
        python3-pip \
        python3-pyqt5.qtopengl \
        wget && \
    # Install spot
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | apt-key add - && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    # Install newer version of castxml than is available via apt-get:
    wget -q -O- https://data.kitware.com/api/v1/file/5b68c2c28d777f06857c1f48/download | tar zxf - && \
    # Install pypy3
    wget -q -O- https://bitbucket.org/pypy/pypy/downloads/pypy3-v6.0.0-linux64.tar.bz2 |tar jxf - && \
    pip3 install pygccxml pyplusplus PyOpenGL-accelerate
COPY . /omplapp
WORKDIR /build
RUN cmake \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCASTXML=/castxml/bin/castxml \
        -DPYPY=/pypy3-v6.0.0-linux64/bin/pypy3 \
        /omplapp && \
    make update_bindings -j `nproc` && \
    make -j `nproc` && \
    make install

FROM ubuntu:bionic
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        build-essential  \
        cmake \
        freeglut3-dev \
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
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | apt-key add - && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev

COPY --from=builder /usr /usr
RUN useradd -ms /bin/bash ompl
USER ompl
WORKDIR /home/ompl
