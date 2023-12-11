# Mobile-Robot-With-Manipulator

### A mobile robot with UR5 arm mounted on top of it for picking and placing operation

## for simulating the above mentioned task
- copy the package inside ros2 workspace and name it to car
- source the workpace
```bash
# build the workspace using
- colcon build
# in another terminal run the below command for launching the gazebo simulation
- ros2 launch car gazebo.launch.py
# Open another terminal and run for picking and placing operation
- ros2 run car pick_place.py
```
