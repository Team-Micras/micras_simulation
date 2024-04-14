ARG ROS_DISTRO

FROM osrf/ros:${ROS_DISTRO}-desktop as base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-ros-gz

RUN mkdir -p /micras_ws/src/micras_simulation
WORKDIR /micras_ws

COPY . src/micras_simulation/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCOMPILE_TESTS=OFF

COPY docker/ros_entrypoint.sh /
RUN echo "source /ros_entrypoint.sh" >> /etc/bash.bashrc

FROM base as sim

RUN rm -rf src/

FROM base as test

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCOMPILE_TESTS=ON

RUN rm -rf src/
