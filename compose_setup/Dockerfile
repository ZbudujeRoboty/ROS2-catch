FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY ros2_ws ros2_ws/

RUN sudo apt update && \
    source /opt/ros/foxy/setup.bash && \
    #sudo apt install python3-rosdep && \
    #sudo rosdep init && \
    #rosdep update && \
    #rosdep install --from-paths src --ignore-src --rosdistro foxy -y %% \
    sudo apt install -y ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-msgs ros-foxy-turtlebot3-msgs ros-foxy-dynamixel-sdk ros-foxy-hls-lfcd-lds-driver && \
    mkdir -p ~/turtlebot3_ws/src && \
    cd ~/turtlebot3_ws/src && \
    git clone -b foxy devel https://github.com/ROBOTIS-GIT/turtlebot3 && \
    git clone -b foxy devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations && \
    cd ~/turtlebot3_ws && \
    colcon build && \
    cd ~/ros2_ws && \
    colcon build

COPY compose_setup/ros_entrypoint.sh /

ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD ["bash"]