# ROS2-service-for-MST
## Description:
<par>
This repository contains a ROS2 service for searching the minimum spanning tree of the graph. The Request message contains string: sequence of edges in form «X-Y», where X and Y are nodes of thegraph. Elements are separated with whitespace.
Response message contains string with edges of the tree in form «A-B B-C etc.»

**example:**

Request
String «A-B A-C B-E E-C E-D E-H D-F D-G G-H»

Response
String «A-C C-E E-B E-H E-D D-F D-G»
  
  
## Algorithm:
  In order to find the MST for a given graph we have used *Kruskal’s Minimum Spanning Tree* algorithm and we used **C++** for coding. for more details about the algorithm you can check this [Link](https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-using-stl-in-c/)

## In order to run the code in your device you can follow the following steps:
  
1. Pull the following docker if you have ubuntu version 20:
```
docker pull osrf/ros:foxy-desktop
```
2. Clone this repo into your device:
```
git clone https://github.com/SibaIssa/ROS2-service-for-MST.git
```
3. Now we need to run our docker and we will name it **mydocker**, and be careful to change the **usr** in the following path with your device's username:
```
sudo docker run -it -v /home/usr/Task:/home/usr/Task --name mydocker osrf/ros:foxy-desktop
```
4. Source the */=*.bash** files
```
source /home/siba/Task/task_ws/install/setup.bash 
```
```
source /opt/ros/foxy/setup.bash
```
5. The following two commands are necessary if you want to execute your docker in more than one terminal so you don't need to source your **.bash** files each time:
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
```
echo "source /home/siba/Task/task_ws/install/setup.bash ">> ~/.bashrc
```
6. Now we are ready to run our **Server**:
```
cd /home/usr/Task/task_ws
```
```
ros2 run mst_srvcli server
```
7. Open new terminal, and execute your **mydocker**:
```
sudo docker exec -ti mydocker bash
```
8. Then run the **Client**:
```
cd /home/usr/Task/task_ws
```
```
ros2 run mst_srvcli client
```
---
### note: for any inconvenience you can close the docker by using the following statement, then do the steps again:
```
sudo docker rm -f mydocker
```


