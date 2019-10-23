sudo apt install terminator -y
sudo apt-get install zsh curl git -y
sh -c "$(curl -fsSL https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh)" -y
curl -sSL git.io/jovial | sudo bash -s $USER
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
mkdir -p ~/doris_ws/src
cd ~/doris_ws
catkin_make
source devel/setup.zsh
cd src
git clone https://github.com/ros-perception/openslam_gmapping
cd ..
catkin_make
cd src
git clone https://github.com/ros-perception/slam_gmapping
cd ..
catkin_make
sudo apt-get install ros-melodic-amcl -y
sudo apt-get install ros-melodic-move-base -y
sudo apt-get install ros-melodic-sick-scan -y
sudo apt-get install ros-melodic-map-server -y
cd ~/doris_ws/src

