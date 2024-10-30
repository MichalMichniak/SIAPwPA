# SIAPwPA - DRL 

# pierwsze budowanie:
colcon build 
source install/setup.bash
ros2 launch simple_example example.launch.py



w nowym terminalu 
ros2 run CameraDataAcquisition cameraDataAcquisition 
W rvision: 
Add / By topic i tam dodajemy topici które chcemy podejrzeć

## How to run Gazebo Simulation
Type in terminal:
- ros2 launch simple_example example.launch.py

## All the important topics
Camera sensor:
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/front_camera_sensor/image
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/right_camera_sensor/image
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/left_camera_sensor/image
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/back_camera_sensor/image

Laser sensor: 
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/center_laser_sensor/scan\
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/center_laser_sensor/scan/points

## Architecture of the system
TODO

## CameraDataAcquisition node
Type in terminal to run:
- ros2 run CameraDataAcquisition cameraDataAcquisition

For now it just collects data from front camera as an example.
