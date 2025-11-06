ros_distro="noetic"

sudo apt-get install -y ros-${ros_distro}-move-base*
sudo apt-get install -y ros-${ros_distro}-industrial-robot-status-interface
sudo apt-get install -y ros-${ros_distro}-rospy-message-converter
sudo apt-get install -y ros-${ros_distro}-industrial-robot-status-interface
sudo apt-get install -y ros-${ros_distro}-ur-msgs
sudo apt-get install -y ros-${ros_distro}-pass-through-controllers
sudo apt-get install -y ros-${ros_distro}-ur-client-library
sudo apt-get install -y ros-${ros_distro}-rosbridge-server
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
