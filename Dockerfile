FROM ros:noetic

# install build tools
RUN apt-get update && apt-get install -y \
      python3-catkin-tools \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

ADD requirements.txt /requirements.txt

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

ADD catkin_ws /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make; . ~/catkin_ws/devel/setup.bash'