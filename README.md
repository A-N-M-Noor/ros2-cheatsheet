# ros2-cheatsheet

## [Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Check if you have UTF8
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

### Setup the sources
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

### Update and upgrade
```bash
sudo apt update
sudo apt upgrade
```

### Install ROS2
```bash
sudo apt install ros-humble-desktop
```

### Add Path to Environment
Open /.bashrc by using
```bash
gedit ~/.bashrc
```
Add this line at the end
```bash
source /opt/ros/humble/setup.bash
```

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
Check if a package exists
```bash
ros2 pkg executables <package_name>
```
```bash
ros2 pkg executables turtlesim
```
