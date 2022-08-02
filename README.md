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
  In order to find the MST for a given graph we have used **Kruskal’s Minimum Spanning Tree** algorithm and we used **C++** for coding. for more details about the algorithm you can check this [Link](https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-using-stl-in-c/)

## In order to run the code in your device you can follow the following steps:
  
**1- pull offical docker container for ros2 foxy**
``` 
  $  docker pull osrf/ros:foxy-desktop
```
**2- download my worksapce**
```
$  git clone https://github.com/SibaIssa/ROS2-service-for-MST.git
```
**3- enter docker**
```
$  sudo docker run -it -v <dir_to_cloned_repo>/ROS2-service-for-MST/task_ws:/home/usr/Task --name mydocker osrf/ros:foxy-desktop
```
**4- go to folder**
```
$ cd home/usr/Task
```
**5- build work space**
```
$  colcon build --packages-up-to mst_srvcli
```
**6-  source for our terminal and other terminals**
```
$ source /home/usr/Task/install/setup.bash && source /opt/ros/foxy/setup.bash
```
```
$ echo "/home/usr/Task/install/setup.bash" >> ~/.bashrc && echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
**7- open another terminals**
```
$ sudo docker exec -ti mydocker bash
```
In case of source permission denied use the following
```
$ source home/usr/Task/install/setup.bash
```
**8- Terminal 1**
```
$  ros2 run mst_srvcli server
```
**9- Terminal 2**
```
$  ros2 run mst_srvcli client A B
```
**10- Enter the graph**
```
  A-B A-C B-E E-C E-D E-H D-F D-G G-H #your graph
```

---
### note: for any inconvenience you can close the docker by using the following statement, then do the steps again:
```
sudo docker rm -f mydocker
```


