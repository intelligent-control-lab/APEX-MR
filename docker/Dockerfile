# Use ROS Noetic Robot base image
FROM osrf/ros:noetic-desktop-full-focal

# Install RViz and other dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    libopencv-dev \
    python3-opencv \
    python3-catkin-tools \
    python3-pip \
    ros-noetic-moveit \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install software-properties-common and wget
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install PyYAML
RUN pip3 install pyyaml

# INSTALL Rviz and Moveit Visual, and control tools
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz-visual-tools \
    ros-noetic-moveit-visual-tools \
    ros-noetic-ros-control \
    && rm -rf /var/lib/apt/lists/*

    
# Install Gurobi 11.0.2
RUN wget https://packages.gurobi.com/11.0/gurobi11.0.2_linux64.tar.gz && \
tar -xvf gurobi11.0.2_linux64.tar.gz && \
mv gurobi1102 /opt/gurobi && \
rm gurobi11.0.2_linux64.tar.gz

# Set Gurobi environment variables
ENV GUROBI_HOME="/opt/gurobi1102/linux64"
ENV PATH="${PATH}:${GUROBI_HOME}/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"

RUN pip3 install gurobipy

# Set Gurobi license environment variables
ARG GUROBI_WLSACCESSID
ENV GUROBI_WLSACCESSID=${GUROBI_WLSACCESSID}
ARG GUROBI_WLSSECRET
ENV GUROBI_WLSSECRET=${GUROBI_WLSSECRET}
ARG GUROBI_LICENSEID
ENV GUROBI_LICENSEID=${GUROBI_LICENSEID}

# Create Gurobi license file
RUN echo "WLSACCESSID=${GUROBI_WLSACCESSID}\nWLSSECRET=${GUROBI_WLSSECRET}\nLICENSEID=${GUROBI_LICENSEID}" > /opt/gurobi/gurobi.lic

# Install pypulp, pandas, numpy in pip
RUN pip3 install pulp pandas numpy==1.24.4

# Setup environment
ENV HOME=/root
WORKDIR $HOME

# Create catkin workspace
RUN mkdir -p catkin_ws/src && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; cd catkin_ws; catkin init'

# clone the apex_mr repository
ARG GITHUB_TOKEN
ENV GITHUB_TOKEN=${GITHUB_TOKEN}
RUN git config --global url."https://${GITHUB_TOKEN}@github.com/".insteadOf "https://github.com/"

RUN cd catkin_ws/src \
    && git clone https://github.com/intelligent-control-lab/APEX-MR.git


RUN cd catkin_ws/src \
    && git clone https://github.com/intelligent-control-lab/Robot_Digital_Twin.git \
    && cd Robot_Digital_Twin \
    && git checkout apexmr-release

# build the workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd catkin_ws; catkin build -DCMAKE_BUILD_TYPE=Release'

# Source ROS setup file
RUN echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc && \
    echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

# Set the entrypoint to the workspace
WORKDIR $HOME/catkin_ws
ENTRYPOINT ["/bin/bash"]
