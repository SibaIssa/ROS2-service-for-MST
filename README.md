# ROS2-service-for-MST
---
This repository contains a ROS2 service for searching minimum spanning tree of the graph. In order to run the code on your device you can follow the following steps:
---
### Pull the following docker if you have ubuntu version 20:
```
docker pull osrf/ros:foxy-desktop
```
### Clone this repo into your device:
```
git clone https://github.com/SibaIssa/ROS2-service-for-MST.git
```
---
### Now we need to run our docker and we will name it *mydocker*, and be carefull to change the *usr* in the following path with your device's username: 
```
sudo docker run -it -v /home/usr/Task:/home/usr/Task --name mydocker osrf/ros:foxy-desktop
```
source the *.bash* files
```
source /home/siba/Task/task_ws/install/setup.bash 
```
```
source /opt/ros/foxy/setup.bash
```
### The following two commands are necessary if you want to execute your docker in more than one terminal so you don't need to source your *.bash* files each time:
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
```
echo "source /home/siba/Task/task_ws/install/setup.bash ">> ~/.bashrc
```
---
### Now we are ready to run our server:
```
cd /home/usr/Task/task_ws
```
```
ros2 run mst_srvcli server
```
### Open new terminal, and execute your *mydocker*:
```
sudo docker exec -ti mydocker bash
```
### Then run the *Client*:
```
cd /home/usr/Task/task_ws
```
```
ros2 run mst_srvcli client
```
---
### note: for any missconvinient you can close the docker by using the following statement, then do the steps again:
```
sudo docker rm -f mydocker
```


