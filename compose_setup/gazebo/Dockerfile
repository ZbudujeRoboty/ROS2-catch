FROM gazebo

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY ros2_ws ros2_ws/

RUN cd ros2_ws && \
    source /usr/share/gazebo/setup.bash

#CMD ["gzserver", "my-gazebo-app-args"]

#COPY gzserver_entrypoint.sh /

#ENTRYPOINT [ "/gzserver_entrypoint.sh" ]

CMD [ "bash" ]