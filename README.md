# SIAPwPA - DRL 

# pierwsze budowanie:
```
colcon build 
source install/setup.bash
ros2 launch simple_example example.launch.py
```

W NOWYM TERMINALU: 
```
ros2 run CameraDataAcquisition cameraDataAcquisition 
gz topic -e  -t /keyboard/keypress
```
# W rvision: 
Add / By topic i tam dodajemy topici które chcemy podejrzeć

## How to run Gazebo Simulation
Type in terminal:
```
ros2 launch simple_example example.launch.py
```
##

## All the important topics
Camera sensor:
- /world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/front_camera_sensor/image


## Uczenie modelu:
W głównym folderze:
```
colcon build 
source install/setup.bash
export ROS_DOMAIN_ID=<id>
ros2 launch simple_example example.launch.py
```
w nowym terminalu:
```
export ROS_DOMAIN_ID=<id>
```
oraz w miejscu gdzie jest model do trenowania należy uruchomić skrypt [simple_example/simple_example/sodoma_env.py](simple_example/simple_example/sodoma_env.py). Model zapisze się pod nazwą model1
Gdzie \<id\> to wartość ROS_DOMAIN_ID z zakresu 0 and 231

## Uruchomienie nauczonego modelu

W głównym folderze:
```
colcon build 
source install/setup.bash
export ROS_DOMAIN_ID=<id>
ros2 launch simple_example example.launch.py
```
w nowym terminalu:
```
export ROS_DOMAIN_ID=<id>
```
oraz w miejscu gdzie jest model do trenowania należy uruchomić skrypt 
[simple_example/simple_example/vehicle_control.py](simple_example/simple_example/vehicle_control.py)
Gdzie \<id\> to wartość ROS_DOMAIN_ID z zakresu 0 and 231
