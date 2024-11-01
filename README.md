# ros2-cheatsheet

## [Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Install pip3
```bash
sudo apt install python3-pip
```

### Install git
```bash
sudo apt install git-all
```

---

### Install ROS2 Humble, ROSDEP and Colcon

#### Check if you have UTF8

```bash
locale
```
If you don't have it,
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
Then verify again.

#### Setup the sources
Enable **Ubuntu Universe Repository**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add ROS2 GPG Key
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add the repository to the source list
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Update and upgrade
```bash
sudo apt update
sudo apt upgrade
```

#### Install ROS2
```bash
sudo apt install ros-humble-desktop
```

#### Install Colcon
```bash
sudo apt install python3-colcon-common-extensions
```

#### Add Path to Environment
Open ` .bashrc ` by using
```bash
gedit ~/.bashrc
```
Add these line at the end
```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
Save & quit.

#### Install rosdep
```bash
sudo apt install python3-rosdep2
```
```bash
rosdep update
```
---

## [Install f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)

```bash
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym
pip3 install -e .
```

### Install the simulation
* Create a workspace in the home directory and clone the simulation
```bash
cd ~
mkdir -p sim_ws/src
cd sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros
```
* Go to ` /config/sim.yaml ` and change the map path. It may be `"~/sim_ws/src/f1tenth_gym_ros/maps/levine"`

* Move to the ws directory and install dependencies.
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
* Build the workspace
```bash
colcon build
```
* Add the local ` setup.bash ` to the .bashrc file.
```bash
gedit ~/.bashrc
```
Add the source at the end:
```bash
source ~/sim_ws/install/local_setup.bash
```

#### Launch the simulation
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

---

### Install VSCode
```bash
sudo snap install --classic code
```

---
---
---

## Useful Commands
View the **rqt graph**
```bash
ros2 run rqt_graph rqt_graph
```
or just
```bash
rqt_graph
```

---

List all the packages
```bash
ros2 pkg list
```

List all the active nodes
```bash
ros2 node list
```

Check if a package exists
```bash
ros2 pkg executables <package_name>
```
```bash
ros2 pkg executables turtlesim
```

---

Run a node
```bash
ros2 run <pkg> <node>
```
```bash
ros2 run turtlesim turtlesim_node
```

View node info
```bash
ros2 node info <node>
```

---

View topic info
```bash
ros2 topic info <topic>
```

Print topic in the terminal
```bash
ros2 topic echo <topic>
```

---

View message interface
```bash
ros2 interface show <interface>
```

## Creating a ROS2 Package
### Make the workspace
Make a new directory, move into the directory, and create a src sub-directory.
```bash
mkdir <ros2_ws>
cd <ros2_ws>
mkdir src
```

### Create the package
Move into the ` src ` directory and run
```bash
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
```

### Build the package
Build using colcon (Must be from the workspace directory)
```bash
colcon build
```
or
```bash
colcon build --symlink-install
```

If the build shows error, downgrade python setuptools to 58.2.0
```bash
pip3 install setuptools==58.2.0
```

Add the package to the ` .bashrc ` file
```bash
source ~/<ros2_ws>/install/setup.bash
```
