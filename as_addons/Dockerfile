# syntax=docker/dockerfile:1
ARG ROS_DISTRO=humble
ARG CMAKE_BUILD_TYPE=Release
ARG BUILD_REF=unknown
ARG INSTALL_DIR=/opt/autoware.universe


# ================================
# STAGE: Prepare base image
# ================================
FROM ros:${ROS_DISTRO} as base
SHELL [ "bash", "-eo", "pipefail", "-c" ]

ARG TZ=UTC
ARG DEBIAN_FRONTEND=noninteractive

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV CCACHE_DIR=/ccache
ENV CCACHE_UMASK=000

RUN mkdir -p /work/autoware_universe_ws
WORKDIR /work/autoware_universe_ws

RUN echo "BUILD(base): Installing basic tools..." \
  && apt-get update -y && apt-get install -y \
    ccache \
    curl \
    gnupg \
    openssh-client \
    tzdata \
    python3-pip \
    nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*

# Use ccache (make it appear in path earlier then /usr/bin/gcc etc)
RUN for p in gcc g++ cc c++; do ln -vs /usr/bin/ccache /usr/local/bin/$p;  done

# During build CCACHE_DIR directory is mapped from a docker buildkit cache, but we create
# an empty directory just in case, to be used when ccache is used in derived container and
# no ccache volume is mapped.
RUN mkdir -p $CCACHE_DIR && chmod 777 $CCACHE_DIR


# ================================
# STAGE: Download sources
# ================================
FROM base as downloader

# Force sources redownload on every changed TIMESTAMP
ARG TIMESTAMP

# Setup ssh and download external source repositories
COPY build_depends_humble.repos /work/autoware.universe.build_depends_humble.repos
RUN --mount=type=ssh \
  echo "BUILD(downloader): Downloading external sources..." \
  && mkdir -p -m 600 $HOME/.ssh \
  && ssh-keyscan gitlab.asystems.io >> ~/.ssh/known_hosts \
  && mkdir src \
  && vcs import --recursive src < /work/autoware.universe.build_depends_humble.repos \
  && rm -rf ~/.ssh/

# Add current repository to sources
COPY . ./src/autoware.universe

ARG INSTALL_DIR

# Gather information about required dependendencies
RUN echo "BUILD(downloader): Resolving dependencies..." \
  && mkdir -p ${INSTALL_DIR} \
  && apt-get update -y \
  && rosdep update \
  && rosdep install --from-paths src --ignore-packages-from-source -y --reinstall \
    --skip-keys "awapi_awiv_adapter autoware_iv_internal_api_adaptor autoware_iv_external_api_adaptor eagleye_rt eagleye_gnss_converter eagleye_geo_pose_fusion" \
    --simulate | sort -b -k 4 > ${INSTALL_DIR}/ros-deps \
  && rm -rf /var/lib/apt/lists/* \
  && md5sum ${INSTALL_DIR}/ros-deps

# Remove volatile .git data to enable docker build cache in next stage.
RUN find src -type d -name '.git' -exec rm -rf {} +


# ================================
# STAGE: Install depdendencies and build sources
# ================================
FROM base as builder

ARG INSTALL_DIR

COPY --from=downloader ${INSTALL_DIR}/ros-deps ${INSTALL_DIR}/ros-deps
RUN --mount=type=cache,target=/ccache/ \
  echo "BUILD(builder): Install dependencies..." \
  && apt-get update -y \
  && bash -e ${INSTALL_DIR}/ros-deps \
  #FIX FOR missing https://github.com/ros2/rclcpp/issues/2249
  && apt-get upgrade -y \
  && rm -rf /var/lib/apt/lists/*

ARG CMAKE_BUILD_TYPE
ARG ROS_DISTRO

COPY --from=downloader /work/autoware_universe_ws /work/autoware_universe_ws
RUN --mount=type=cache,target=/ccache/ \
  echo "BUILD(builder): Building autoware.universe for ROS_DISTRO=$ROS_DISTRO with CMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE..." \
  && source /opt/ros/$ROS_DISTRO/setup.bash \
  && colcon build --install-base ${INSTALL_DIR} --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE" \
  && ccache -s

RUN echo "BUILD(builder): Remove temporary build files..." \
  && rm -rf build log

ARG BUILD_REF

COPY as_addons/autoware.universe.readme.md /work/autoware.universe.readme.md
RUN sed -i -e "s|%%ROS_DISTRO%%|${ROS_DISTRO}|g" -e "s|%%BUILD_REF%%|${BUILD_REF}|g" /work/autoware.universe.readme.md


# ================================
# STAGE: Prepare final image, with single layer
# ================================
FROM ros:${ROS_DISTRO} as final

COPY --from=builder / /
WORKDIR /work

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
