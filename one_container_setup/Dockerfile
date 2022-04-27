FROM osrf/ros:foxy-desktop

SHELL ["/bin/bach", "-c"]

WORKDIR /app

COPY ros2_ws ros2_ws/

RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY one_container_setup/ros_entrypoint.sh /

ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD [ "bash" ]