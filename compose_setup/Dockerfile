FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY ros2_ws ros2_ws/

RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    #sudo apt install python3-rosdep && \
    #sudo rosdep init && \
    #rosdep update && \
    #rosdep install --from-paths src --ignore-src --rosdistro foxy -y %% \
    sudo apt install ros-foxy-gazebo-msgs && \
    colcon build

COPY compose_setup/ros_entrypoint.sh /

ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD ["bash"]