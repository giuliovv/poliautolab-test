FROM ros:noetic

# install build tools
RUN apt-get update && apt-get install -y \
      python3-catkin-tools \
      python3-pip \
      git-all \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /requirements.txt

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://192.168.114.211:11311;'
#  cd ~/catkin_ws; catkin_make; . ~/catkin_ws/devel/setup.bash'