# ROS2-service-for-MST
This repository contains a ROS2 service for searching minimum spanning tree of the graph.
---
in order to run the code on your device you can follow the following steps:

```
docker pull osrf/ros:foxy-desktop
```
```
git clone https://github.com/SibaIssa/ROS2-service-for-MST.git
```

---
```
sudo docker run -it -v /home/usr/Task:/home/usr/Task --name mydocker osrf/ros:foxy-desktop
```

```
source /home/siba/Task/task_ws/install/setup.bash 
```
```
source /opt/ros/foxy/setup.bash
```
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
```
echo "source /home/siba/Task/task_ws/install/setup.bash ">> ~/.bashrc
```
```
cd /home/usr/Task/task_ws
```
```
ros2 run mst_srvcli server
```
- open new terminal 
```
sudo docker exec -ti mydocker bash
```
```
cd /home/usr/Task/task_ws
```
```
ros2 run mst_srvcli client
```


