# Launch files
```sh
$ roslaunch myrobot_gazebo myrobot_world.launch 
$ roslaunch myrobot_navigation nav_stack.launch  
```
# Cliff Sensors
1. Front
2. Back
3. Left 30 deg
4. Left 50 deg
5. Right 30 deg
6. Right 50 deg

# Cliff data configuration parameters
- cliff_mark_threshold - Threshold for IR ranges above which to be considered as cliffs.
- cliff_mark_distance - Range that should be published if a cliff is detected.

