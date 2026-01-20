### Aeplanner dependencies

- catkin_simple should be cloned on ws src

git clone https://github.com/catkin/catkin_simple.git

- pigain:

pip install rtree
sudo apt-get install libspatialindex-dev

- rpl fails to compile:

on CMakeLists.txt change C++ to use v14

- actionlib_tools to use actions

apt install -y ros-noetic-actionlib-tools

rosrun actionlib_tools axclient.py /aeplanner/make_plan


