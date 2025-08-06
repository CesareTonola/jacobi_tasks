FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

ENV PATH=/opt/openrobots/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig
ENV LD_LIBRARY_PATH=/opt/openrobots/lib
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages
ENV CMAKE_PREFIX_PATH=/opt/openrobots
ENV MPLBACKEND=TkAgg

RUN apt-get update && apt-get install -y \
    git \
    python3 \
    python3-pip \
    python3-dev \
    python3-tk \
    cmake \
    build-essential \
    libeigen3-dev \
    libgl1-mesa-dev \
    libglfw3-dev \
    libassimp-dev \
    x11-apps \
    libx11-dev \
    libxi-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxinerama-dev \
    pybind11-dev \
    lsb-release \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*


# Add robotpkg repo and install Pinocchio
RUN mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | tee /etc/apt/keyrings/robotpkg.asc > /dev/null && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && \
    apt-get install -y robotpkg-py310-pinocchio && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip==25.2 && pip3 install \
    numpy==1.26.4 \
    scipy==1.15.3 \
    matplotlib==3.10.5 \
    open3d==0.19.0 \
    meshcat \
    meshcat-shapes==1.0.0 \
    colcon-common-extensions


WORKDIR /jacobi_ws

RUN git clone https://github.com/CesareTonola/jacobi_tasks.git src/jacobi_tasks

RUN colcon build --symlink-install

CMD ["bash", "-c", "source install/setup.bash && exec bash"]

