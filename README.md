# nav2_ompl

ROS2 package for demonstrating the usage of OMPL with ROS2 [Nav2 stack](https://navigation.ros.org/) and [Turtlebot4](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html). The package implements a global planner plugin that uses OMPL's [BITStar planner](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1BITstar.html) and [DubinsStateSpace](https://ompl.kavrakilab.org/classompl_1_1base_1_1DubinsStateSpace.html). Here is an example of the output:

![ompl_planner_demo](ompl_planner_demo.gif)


# Usage Instructions

Before using this package you should install the following:    

* [ROS2](https://docs.ros.org/en/rolling/Releases.html) (this package was tested on Galactic)
* [Nav2](https://navigation.ros.org/)
* [Turtlebot4 packages](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html)

1. Create new workspace, clone packages needed:
```
# Create workspace
mkdir -p ~/nav2_ompl_ws/src
cd nav2_ompl_ws/src/
git clone https://github.com/Omar-Elmofty/nav2_ompl.git
git clone https://github.com/turtlebot/turtlebot4.git
# Need to checkout the correct ros distro branch for turtlebot4
cd turtlebot4/
git checkout <your_ros_distro>
```

2. Now we'll need to edit some params in the `turtlebot4_navigation` package, open the file `turtlebot4/turtlebot4_navigation/config/nav2.yaml` and replace the `planner_server` block with:
```
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_ompl/OMPLPlanner" # Using our new plugin here

```
Note that the above is kinda of hack, technically we should be creating a new planner plugin name other than the `GridBased` one, but since the `GridBased` name is the default name throughout the turtlebot4 stack, so it's easier to stick with it and change the `plugin` variable as shown above.

3. Build the workspace
```
# Build workspace
cd ~/nav2_ompl_ws
colcon build
```

4. Launch turtlebot4 simulation, more information about launching sim can be found [here](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html), but you could just use the below commands. Assumption here is that you've already mapped the environment and saved the map files, which you will need to load in the second command below when bringing up the nav2 stack (the arg `map:=your_map.yaml`).
```
# Open a new terminal, and run
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav2:=false slam:=off localization:=false rviz:=true

# Open a new terminal, and run
cd ~/nav2_ompl_ws
source install/setup.bash # Here we source the workspace to apply the changes we made
ros2 launch turtlebot4_navigation nav_bringup.launch.py map:=your_map.yaml localization:=true slam:=off
```
5. Issue goals and see the OMPL planner in action!
