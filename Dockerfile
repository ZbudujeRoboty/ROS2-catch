FROM python

WORKDIR /app

# RUN npm install ./src/husarion_mini_project/husarion_mini_project/package.xml

COPY . /app

#RUN export TURTLEBOT3_MODEL=burger

CMD ["python3", "./src/husarion_mini_project/husarion_mini_project/turtlebot3_spawner.py"]
