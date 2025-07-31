ARG TAG=humble
FROM ghcr.io/tesseract-robotics/tesseract_ros2:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Run apt update 
RUN apt update -y -qq && apt install -y -qq sudo python3-pip

# Install pyquaternion
RUN python3 -m pip install pyquaternion

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/snp_automate_2023
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp_automate_2023 \
  apt update -y -qq \
  && source /opt/tesseract_ros2/install/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/snp_automate_2023/docker/dev/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
    --skip-keys libvtk

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp_automate_2023 \
  source /opt/tesseract_ros2/install/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args \
  && rm -rf build log

COPY --chmod=755 docker/launch_snp_automate_2023.sh /opt/launch_snp_automate_2023.sh
COPY --chmod=755 docker/calibration.entrypoint.sh /opt/calibration.entrypoint.sh
ENTRYPOINT ["/opt/launch_snp_automate_2023.sh"]
