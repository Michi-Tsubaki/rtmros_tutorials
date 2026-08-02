# nextage_tutorials

## jsk-nextage tutorials

### Setup

```bash
cd ~
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/Michi-Tsubaki/rtmros_tutorials.git -b ros2
sudo apt update
sudo apt upgrade
rosdep update
rosdep install --from-path src -iry --skip-keys "realsense2_camera realsense2_description"
colcon build --packages-up-to nextage_tutorials --symlink-install
```


### Launch

#### Terminal 1 (`ros1_bridge`)

```bash
cd ~/colcon_ws/src/rtm_tutorials/nextage_tutorials/launch
./bridge.sh
```

#### Terminal 2 (cameras and force torque sensors)

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch nextage_tutorials nextage.launch.xml 
```