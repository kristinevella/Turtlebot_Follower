# Turtlebot Robot Following Each Other
## Project Description
Using the learnt and state-of-art control algorithm to control the Turtlebot to follow the path of the 
guider  Turtlebot  in  front.  Both  depth  images  and  RGB  images  can  be  used  to the  control.  Special 
designed artificial markers or patterns can be put on the back of guider.

**Target:** The  Turtlebot  follows  the  guider  Turtlebot  in  front  moved  with  real  Turtlebot  Robots/in 
Turtlebot Simulator with a certain distance. 

## Requirements
### System
- Ubuntu 18.04  
- ROS Melodic or Noetic 
- MATLAB 2022a
#### For use with Real Robots (not needed for simulation)
- 2x Turtlebot3 Waffle Pi
- 1x Intel® RealSense™ R200 camera
- Printed April Tag from tag36h11 family (9x9cm) mounted to leading robot

### Dependencies
- ROBOTIS-GIT Turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3
- Intel RealSense - https://github.com/intel-ros/realsense.git

## Running the Project
1. Clone this repository to the /src directory of your workspace
2. For real robot: Connect to via ROS - https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
3. For simulation: Run launch file `roslaunch turtlebot_follower main.launch`
4. Open and run turtlebot_follower.m in MATLAB `tf = turtlebot_follower`
5. Call FollowTheLeader function to start `tf.FollowTheLeader`
6. Control the leading robot using the keyboard with `rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot2/cmd_vel`

## Collaborators
- Kristine Vella - https://github.com/kristinevella
- Charlize Petersen - https://github.com/chanpe207
- Kenny Tafianoto - https://github.com/KennyTafianoto
