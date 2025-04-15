sudo apt update && sudo apt install libgtk-3-dev libapriltag-dev

pip install --upgrade packaging --user

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export COLCON_WS=~/bwi_ros2' >> ~/.bashrc 
mkdir bwi_ros2
cd bwi_ros2
mkdir src

cd ~/bwi_ros2
git clone https://github.com/utexas-bwi/serial_for_ros2.git

cd ~/bwi_ros2/serial_for_ros2/
mv serial ..
cd ..
cd serial
rm -rf build
mkdir build
cd build
cmake ..
make

cd ~/bwi_ros2/src
git clone --branch humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
git clone https://github.com/utexas-bwi/bwi_ros2_common.git

# On a V2
cd ~/bwi_ros2/src
git clone https://github.com/utexas-bwi/libsegwayrmp_ros2.git
mv libsegwayrmp_ros2/ libsegwayrmp
git clone https://github.com/utexas-bwi/segway_rmp_ros2.git

# Build
cd ~/bwi_ros2
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash