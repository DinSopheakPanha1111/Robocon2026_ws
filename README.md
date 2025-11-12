**NOTE : This is the robot_description file along with ros2 controller plugin for mecanum wheel robot (for now) by DIN SOPHEAK PANHA (ONEDIN Tech).**
1. Clone the project
```
git clone https://github.com/DinSopheakPanha1111/Robocon2026_ws.git
```
2. Build and source the file
```
cd Robocon2026_ws
colcon build
source install/setup.bash
```
3. To launch the robot in rviz
```
ros2 launch r2 robot.display.py
```
4. To control the robot
```
ros2 run r2 keyboard_teleop 
```
