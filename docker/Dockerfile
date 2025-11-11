ARG TAG=humble
FROM ghcr.io/ros-industrial-consortium/scan_n_plan_workshop${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractivej

USER root

# Run apt update and install useful utilities like sudo
RUN apt update -y -qq && apt install -y -qq sudo python3-pip

# Install pyquaternion
RUN python3 -m pip install pyquaternion

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/snp_automate_2023
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp_automate_2023 \
  apt update -y -qq \
  && source /opt/snp/install/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/snp_automate_2023/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp_automate_2023 \
  source /opt/snp/install/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log

COPY --chmod=755 docker/launch_snp_automate_2023.sh /opt/launch_snp_automate_2023.sh
COPY --chmod=755 docker/calibration.entrypoint.sh /opt/calibration.entrypoint.sh
ENTRYPOINT ["/opt/launch_snp_automate_2023.sh"]
