FROM ubuntu:jammy

WORKDIR /tmp/build

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    pkg-config \
    openssl \
    libssl-dev \
    libiodbc2 \
    iodbc \
    libpoco-dev \
    libeigen3-dev \
    python3 \
    clang-tidy \
    clang-format \
    cppcheck \
    vim \
    gdb \
    gosu

RUN curl -L https://github.com/Kitware/CMake/archive/refs/tags/v4.2.0.tar.gz -o cmake.tar.gz \
    && tar -xf cmake.tar.gz \
    && cd CMake-4.2.0 \ 
    && ./bootstrap --prefix=/usr --parallel='nproc' \
    && make && make install
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5

RUN apt-get update \
    && apt-get install -y lsb-release \
    && mkdir -p /etc/apt/keyrings \ 
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | tee /etc/apt/keyrings/robotpkg.asc \
    &&  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -qqy robotpkg-py3*-pinocchio
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH 
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

RUN git clone --recurse-submodules https://github.com/Toprak-Efe/libfranka \
    && cd libfranka \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \ 
    && cmake --build build --parallel \
    && cmake --build build --target install

RUN apt-get update && apt-get install -y \
    software-properties-common \
    locales \
    && add-apt-repository universe \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/Europe/Istanbul /etc/localtime \
    && DEBIAN_FRONTEND=noninteractive apt install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \ 
    && apt-get update \
    && apt-get upgrade \
    && apt-get install -y ros-humble-desktop ros-dev-tools



RUN useradd -ms /bin/bash geronimo

COPY . /home/geronimo/franka-workspace

USER geronimo
WORKDIR /home/geronimo

