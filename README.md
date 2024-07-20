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


## Commands related to python package 

- ### Creating a python package named `my_pkg` : 
```
ros2 pkg create my_pkg --build-type ament_python dependencies rclpy
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
