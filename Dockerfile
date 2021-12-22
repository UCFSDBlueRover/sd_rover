ARG BASE_IMG=nvcr.io/nvidia/l4t-base:r32.6.1

FROM ${BASE_IMG}

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# install some prereqs
RUN apt-get update && \
	apt-get install -y --no-install-recommends \
	git \
	cmake \
	build-essential \
	curl \
	wget \
	ca-certificates \
	gnupg2 \
	lsb-release \
	&& rm -rf /var/lib/apt/lists/*

# ROS install
ARG ROS_PKG=ros-base
ENV ROS_DISTRO=melodic

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update && \
	apt install -y --no-install-recommends \
	ros-${ROS_DISTRO}-${ROS_PKG} \
	python-rosdep \
	python-rosinstall \
	python-rosinstall-generator \
	python-wstool \
	build-essential \
	python-rosdep	\
  	python-vcstools \
  	&& rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup entrypoint
WORKDIR /root/ros_workspace
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]