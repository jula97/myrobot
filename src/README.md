# Diyazen Junior

1. First of all clone this repository to your local machine. 

    `$ git clone git@gitlab.com:arimac-robotics/diyazen-junior.git`

2. Install catkin tools in order to build the workspace

    `$ sudo apt-get update`

    `$ sudo apt-get install python-catkin-tools`

3. Install all the dependencies required to run the simulation.

    `$ rosdep update`

    `$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y`

4. Build the workspace

    `$ catkin build`

    You should see the directories build, devel, logs alongside with src. 

5. Source your workspace. First, Navigate to the root of your workspace. 

    `$ source /devel/setup.bash`

6. Start the mapping demo using Diyazen Junior 

    `$ roslaunch diyazen_navigation gazebo_mapping_test.launch`

7. Start the keyboard teleoperation node to manually control the robot.

	`$ roslaunch diyazen_teleop keyboard_teleop.launch `

	In case you happen to have a Logitech F710 or an Xbox 360 controller handy, you could use that instead!

8. Most important step- Enjoy! 






