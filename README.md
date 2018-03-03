# Schunk lwa4p Gazebo Simulation
ROS package for simulating Schunk arm in Gazebo using Moveit!

## How to use this driver
### Install all ros dependecies
Open the terminal and type:
```{r, engine='bash', count_lines}
$ sudo apt install ros-kinetic-moveit
$ rosdep install -y -r schunk_lwa4p schunk_libm5api schunk_lwa4p_moveit
```

### Run Gazebo
Open the terminal and type:
```{r, engine='bash', count_lines}
$ roslaunch schunk_lwa4p sim.launch
```

### Run Moveit!
Open the terminal and type:
```{r, engine='bash', count_lines}
$ roslaunch schunk_lwa4p_moveit demo.launch
```

