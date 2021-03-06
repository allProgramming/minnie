sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-key adv --keyserver hkp://keyservers.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop-full -y

# TODO: move these to package.xml
sudo apt-get install ros-kinetic-xacro -y
sudo apt-get install ros-kinetic-diff-drive-controller -y
sudo apt-get install ros-kinetic-teleop-twist-keyboard -y

sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ${HOME}/minnie/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
