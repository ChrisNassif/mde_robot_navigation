# Notes from Chris

ROS2 Installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html


sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-robot-localization


pip install -r requirements.txt

bash create_udev_rules.sh







# Install Commands for ROS 2, Isaac ROS, Nvblox, and Sample Dataset (from Austen)

All the commands in this section were used to install and setup ROS2, Isaac ROS cuVSLAM, and Nvblox on our Jetson Orin Nano Super. They also download NVIDIA's recorded Galileo rosbag camera dataset. These commands were orignally run by Austen Coogan.

**These commands do not install drivers for the Orbbec Gemini 2 XL camera.** These commands were not for the final live-camera mapping test. They were only used to map the sample rosbag dataset to simulate the mapping without a camera.

## Specifications of Tested Environment

| Item | Tested configuration |
|---|---|
| Computer | NVIDIA Jetson Orin Nano Super, ~8 GB unified memory |
| Operating system | Ubuntu 24.04 (ARM64, Noble) |
| JetPack / L4T | JetPack 7.2.1 / L4T R39.2.1 (base NVIDIA platform) |
| CUDA | CUDA 13.2 reported during platform checks |
| ROS 2 | Jazzy |
| NVIDIA Isaac ROS | Release 4.6, apt packages for cuVSLAM and Nvblox |
| Isaac ROS CLI | 2.5.0 |
| Visualization | Foxglove Bridge, running on Jetson (Foxglove client on separate laptop) |
| Camera input used for test | NVIDIA `galileo_static_3_2` ROSbag dataset, (not a connected Orbbec camera) |

## 1. Prepare Ubuntu's package repositories

**Note:** Jetson was originally set up from an NVIDIA Ubuntu/JetPack image for Ubuntu 24.04.

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg
sudo add-apt-repository -y universe
```

## 2. Add the ROS 2 APT repository and install ROS 2 Jazzy

The commands below fetched the latest release of `ros2-apt-source`.

```bash
export ROS_APT_SOURCE_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F '"' '{print $4}')

curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools
```

Source ROS automatically in new Bash terminals (run once; avoid duplicating the line in `.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

## 3. Create the Isaac ROS workspace

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
echo 'export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"' >> ~/.bashrc
source ~/.bashrc
```

This creates the `ISAAC_ROS_WS` environment variable used by the commands below.

## 4. Add NVIDIA Isaac ROS 4.6 and CUDA package repositories

```bash
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg

echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.6 noble-jetpack main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list

sudo apt update
sudo apt install isaac-ros-cli
```

The CUDA SBSA repository keyring was also installed in the original setup:

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/sbsa/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
```

Initialize the Isaac ROS CLI's native virtual environment and refresh the group:

```bash
sudo isaac-ros init venv
newgrp isaac-ros-cli
```

## 5. Install cuVSLAM and Nvblox packages

Run this installation command:

```bash
sudo apt-get install --allow-downgrades \
  ros-jazzy-isaac-ros-visual-slam \
  ros-jazzy-isaac-ros-nvblox
```

APT also pulled in `ros-jazzy-nvblox-examples-bringup`, `ros-jazzy-nvblox-nav2`, `ros-jazzy-foxglove-bridge` and related libraries automatically.

**Important troubleshooting fix (only if needed):** When installing on our Jetson, the first install attempt unexpectedly failed due to an OpenCV package conflict. This issue was fixed by running these two commands:

```bash
sudo dpkg --remove opencv-licenses
sudo apt-get --fix-broken install --allow-downgrades
```

## 6. Optional: initialize rosdep

These commands were run, but rosdep initialization is not strictly required for this dataset mapping test, however they might be useful during later development.

```bash
sudo rosdep init

sudo curl -fL \
  -o /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml \
  https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/release-4.6/docker/rosdep/extra_rosdeps.yaml

echo 'yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml' \
  | sudo tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list

rosdep update
```

If rosdep was already initialized, don't repeat `sudo rosdep init`.

## 7. Download and extract the sample dataset (NVIDIA's Galileo static ROSbag)

The dataset was downloaded from NVIDIA NGC version **4.0.0**. The selected folder inside the archive is `galileo_static_3_2`.

```bash
sudo apt-get install -y curl jq tar
mkdir -p "$ISAAC_ROS_WS/isaac_ros_assets"

FILE_URL="https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/isaac_ros_nvblox_assets/versions/4.0.0/files/quickstart.tar.gz"

curl -fL --retry 5 -C - \
  -o "$ISAAC_ROS_WS/isaac_ros_assets/quickstart.tar.gz" \
  "$FILE_URL"
```

**The file has a `.tar.gz` name but was actually an uncompressed GNU TAR archive in our successful download.** Using `tar -tzf` produced a `not in gzip format` error. We used `tar -tf` and `tar -xf` instead:

```bash
file "$ISAAC_ROS_WS/isaac_ros_assets/quickstart.tar.gz"

tar -tf "$ISAAC_ROS_WS/isaac_ros_assets/quickstart.tar.gz" \
  | grep -E 'galileo_static|\.mcap$|metadata\.yaml$' | head -n 30

tar -xf "$ISAAC_ROS_WS/isaac_ros_assets/quickstart.tar.gz" \
  -C "$ISAAC_ROS_WS/isaac_ros_assets" \
  isaac_ros_nvblox/galileo_static_3_2/
```

This extracts **only** the static Galileo bag, rather than the entire large NGC asset archive.

As an optional check, you can verify that ROS2 can read the extracted recording:

```bash
ls -lh "$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_nvblox/galileo_static_3_2/"
ros2 bag info "$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_nvblox/galileo_static_3_2"
```

**Observed result:** ~3.9 GiB MCAP bag, 30.224 seconds, 22,331 ROS 2 messages; recorded color/depth/stereo infrared/IMU/TF topics. The archive download itself was ~8.7 GB.

## 8. Runing the dataset mapping demonstration

Open two terminals on the Jetson and run these commands:

### Terminal 1: Launch the Foxglove Bridge

Running the Foxglove bridge on the the Jetson is necessary in order for the Foxglove client on the remote operator laptop to receive the ROS2 mapping topics from the Jetson.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=0.0.0.0 port:=8765
```

On the laptop, use Foxglove to connect to `ws://<JETSON_WIFI_IP>:8765`. Use the Jetson's current IP address.

### Terminal 2: Launch the NVIDIA recorded-data cuVSLAM + Nvblox sample dataset

First start Isaac ROS:

```bash
isaac-ros activate
```

Then play the rosbag dataset (note that it only plays for ~30-60 seconds):

```bash

ros2 launch nvblox_examples_bringup realsense_example.launch.py \
  rosbag:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_nvblox/galileo_static_3_2" \
  mode:=static \
  num_cameras:=1 \
  run_foxglove:=False \
  run_rviz:=False \
  rosbag_args:="--rate 1.0"
```

To replay the dataset, press `CTRL` + `C` and wait several seconds until the terminal allows you to enter another command. At this point you can copy-paste the `ros2 launch` command shown above.

### Useful mapping verification commands

To test if cuVSLAM and Nvblox are working properly, open a 3rd terminal and run these commands during the ~30-60 second window while the rosbag dataset plays:

```bash
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /nvblox_node/static_esdf_pointcloud
ros2 topic info /nvblox_node/static_map_slice
ros2 topic echo --once /nvblox_node/static_map_slice --field width
ros2 topic echo --once /nvblox_node/static_map_slice --field height
ros2 topic echo --once /nvblox_node/static_map_slice --field resolution
```

This test showed cuVSLAM odometry at ~15 Hz during one test, Nvblox ESDF point-cloud output at ~4.7 Hz, and a map slice of 144 × 136 cells at ~0.05 m/cell. A visible 3D map appeared in the Foxglove client on the operator laptop over Wi-Fi. This verifies functional dataset-based processing, **not** real mapping accuracy, a real camera, or any Nav2 navigation.
