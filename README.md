# Schunk lwa4p Gazebo Simulation
ROS package for simulating Schunk arm in Gazebo using Moveit!

## How to use this driver
### Install all ros dependecies
Open the terminal and type:
```{r, engine='bash', count_lines}
$ cd ~/catkin_ws/src/
$ git clone https://github.com/h3ct0r/schunk_lwa4p_gazebo_sim
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i -y
$ catkin_make
```

### Run Gazebo
Open the terminal and type:
```{r, engine='bash', count_lines}
$ roslaunch schunk_lwa4p_moveit sim_gazebo.launch
```

### Run Moveit!
Open the terminal and type:
```{r, engine='bash', count_lines}
$ roslaunch schunk_lwa4p_moveit demo.launch
```

