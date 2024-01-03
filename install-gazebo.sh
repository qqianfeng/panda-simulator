sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
GAZEBO_MAJOR_VERSION=11 ROS_DISTRO=noetic . /tmp/dependencies.sh
echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs sudo apt-get -y install

# Main repository
sudo apt-add-repository ppa:dartsim
sudo apt-get update
sudo apt-get install -y libdart6-dev

# Optional DART utilities
sudo apt-get install -y libdart6-utils-urdf-dev

git clone https://github.com/osrf/gazebo /tmp/gazebo
cd /tmp/gazebo
git checkout gazebo11

mkdir build
cd build

cmake ../
#cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../

make -j4
sudo make install

echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
sudo ldconfig