# NOT READY, DOES NOT WORK
## ROS2-catch
### ROS2 / Gazebo / Docker mini educational project. Turtlebot3 burger catching other randomly spawning robots in Gazebo simulation.

The app contains two python nodes. One for spawning new robots. One for controlling the main robot.
It also uses turtlebot3 gazebo simulation.
It also uses Docker and Docker-compose.

### Docker Engine instalation (Ubuntu):
> https://docs.docker.com/engine/install/

##### Set up the repo:
1. `sudo apt-get update`
2. `sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release`
3. ` curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg`
4. `echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null`
##### Install Docker Engine:
1. ` sudo apt-get update`
2. ` sudo apt-get install docker-ce docker-ce-cli containerd.io`
3. to verify `sudo docker run hello-world`

#### Docker-compose instalation:
1. `sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose`
2. `sudo chmod +x /usr/local/bin/docker-compose`
3. `sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose`
4. to verify: `docker-compose --version`

## Main steps:
First you need to pull Docker ROS2 image 
`docker pull osrf/ros:foxy-desktop`
which is from > https://hub.docker.com/r/osrf/ros/
