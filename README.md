

# Helpful ros2/gazebo commands 

These commands are used while debugging or working with your ros2 package and gazebo world.

---

## Command related to building your package

- ### Installing colcon build tool :
```
sudo apt install python3-colcon-common-extensions
```

- ### sourcing `argcomplete.bash` file to get autocomplete tools of colcon :
```
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

- ### building your package (you have to be in your `ros2_ws/` folder)
```
colcon build
```
*Note 1*: with this build method you have to always build your workspace/package whenever you make any change in your node.    
There is a way that will automatically keep in sync your build environment with your changes so that u don't need to build explictly if u do some changes in your node : 

```
colcon build --symlink-install
```

*Note 2*: In case you get warning while building your package like `SetuptoolsDepreciation warning: setup.py install is depreciated. Use build and pip and other standard based tools.` Just follow the following steps :  
```
pip3 list
pip3 list | grep setuptools
pip3 install setuptools==58.2.0
pip3 list | grep setuptools
source ~/.bashrc
```
downgrade setuptools version from 59.6.0 to 58.2.0.


## Commands related to ros2 package 

- ### Creating a python package named `my_pkg` : 
```
ros2 pkg create my_pkg --build-type ament_python dependencies rclpy
```


## Commands related to ros2 topics

- ### Listing currenlty active ros topics : 
```
ros2 topic list
```

- ### Getting the info about topic (like msg type & publishers/subscribers of the topic): 
```
ros2 topic info /topic_name
```

- ### Listing all currenlty installed ros2 message types (for topics, services & actions) on ros environment : 
```
ros2 interface list
```
These are basically ways to inteface/communicate in a ros environment.   
Or your can also use
```
ros2 interface list | grep msg
```

- ### Visuzlizing the type of content/data present in a message type (be it of a topic or service or action) : 
```
ros2 interface show /topicType
```

- ### Listen/subscribe to a topic on terminal: 
```
ros2 topic echo /topic_name
```


## Commands related to ros2 services
ros2 services are usually to apply/change specific settings of a robot or to do some computations (so that a response only happens when we call that service).

- ### List currenlty active services : 
```
ros2 service list
```

- ### List the message type of a service :
```
ros2 service type /service_name
```

- ### Getting the type of content/data in a msg type of a service :
```
ros2 interface show /service_msg__type
```

- ### Calling a service from terminal : 
```
ros2 service call /service_name /service_msg_type "{arguments_of_service}"
```


## Commands related to tf2 (TF-Transforms of a robot) in ros environment

- ### Install the tf2-tools package of ros2 :
```
sudo apt install ros-humble-tf2-tf2_tools
```

- ### Source your ros2 environment : 
```
source /opt/ros/humble.setup.bash
```

- ### show the current transforms being published/subscribed in ros2 environment :
```
ros2 run tf2_tools view_frames
```
This command will generate a pdf file listing all the transforms being used currently in your current working directory.



## Commands  related to ros2 bag
ROS2 bags are basically way to record ros2 topic's data in a file.    
Make sure you've already created a `bags` folder in your workspace/package to make it more clear. All these commands should be run while your're currenlty in `bags` directory n your terminal.

- ### saving a ros2 topic's echo data in a file :
```
ros2 bags record /topic_name
```
By default the folder name will be rosbag2_YYYY_MM_DD-HH_MM_SS. In case you want to save the file in folder with a specific name, you can use the command
```
ros2 bag record /topic_name -o folder_name
```

- ### recording multiple ros2 topics :
```
ros2 bag record /topic_name1 /topic_name2 -o folder_name
```

- ### recording all the topics in current ros environment : 
```
ros2 bag record -a -o folder_name
```

- ### viewing a particular ros2bag info :
```
ros2 bag info /ros2bag_folder_name
```

- ### playing/publishing a pre-recorded ros2 topic data from it's corresponding saved bag : 
```
ros2 bag play /ros2bag_folder_name
```
Using this you don't need to explicitly publish the data on that node, this bag will publish the pre-recorded data on all those topics whom it recorded and you can see those topics in topic list and echo and stuff. It's basically playing those same topics without your original robot's ros environment.





## Visualizing a urdf
We'll use a `urdf_tutorial` of ros2 to visualize a urdf file with all it's links, joints, movements and all.

- ### First you need to install urdf_tutorial package :
```
sudo apt install ros-humble-urdf_tutorial
```

- ### Source your ros environment : 
```
source /opt/ros/humble/setup.bash
```

- ### launch display file with urdf as the argument :
```
ros2 launch urdf_tutorial display.launch.py model:=absolute_path_to_urdf_file
```




## Commands related to gazebo sim

- ### launching gazebo-sim using terminal : 
```
gz sim
```

- ### Launch gazebo-sim with empty world using command from terminal : 
```
gz sim empty.sdf
```

- ### launching gazebo-sim with empty world using launch file from terminal :
```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```


## Commands related to ros visualization

- ### visualizing rqt graph : 
```
rqt_graph
```
