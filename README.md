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

# Implementacja

Implementacja algorytmu składa się z paru części łączących interfejs gazebo z interfejsem w pythonie za pomocą systemu ROS 2.

![topic](https://github.com/user-attachments/assets/377a5dab-8587-43e7-9f62-87643e7c27b2)

W procesie trenowania algorytmu została wykorzystana biblioteka Stable Baselines oraz do implementacji środowiska biblioteka gym. Jako model wybrano model dedykowany do przetwarzania obrazów - CnnPolicy.

Pierwsza wersja algorytmu sterowania symulacją w środowisku Gazebo została zaimplementowana synchronicznie. Ze względu na ograniczenia techniczne (brak możliwości instalacji pluginu do pauzowania symulacji po określonej liczbie kroków), konieczne było opracowanie mechanizmu alternatywnego z wykorzystaniem zewnętrznego interfejsu Gazebo.

## Problemy w implementacji synchronicznej
1. **Niedeterministyczne czasy odczytu i zadawania sterowania**:
   - Obserwacje modelu były odczytywane w sposób sekwencyjny, co powodowało duże i zmienne opóźnienia.
   - Czasy zadawania sterowania również były niedeterministyczne, co prowadziło do sytuacji, gdzie model pojazdu zmieniał sterowanie w trakcie trwania kroku symulacji.
2. **Niestabilność funkcji zmiany stanu**:
   - Ze względu na wahania czasowe (krok zamiast 0.5 s trwał nawet kilka sekund), funkcja zmiany stanu nie spełniała założeń procesu Markowa, co istotnie utrudniało proces uczenia ze wzmacnianiem.

## Rozwiązanie
Aby zwiększyć deterministyczność i stabilność algorytmu, wprowadzono asynchroniczny model odczytu obserwacji i zadawania sterowania:
- Zastosowano komendy sterujące w bashu do interakcji z Gazebo.
- Asynchroniczność pozwoliła na bardziej precyzyjne sterowanie oraz zmniejszenie odchylenia standardowego funkcji zmiany stanu.

![asynch](https://github.com/user-attachments/assets/5990a235-350a-4532-b739-fad0b0386c8e)

## Funkcjonalności

- **Wielowątkowość:**  
  Wykorzystanie mutexów i eventów do zarządzania wątkami, zapewniając płynny przepływ danych.
  
- **Integracja z ROS:**  
  Środowisko korzysta z tematów ROS do komunikacji w czasie rzeczywistym.

- **Proximal Policy Optimization (PPO):**  
  Implementacja stabilnego algorytmu uczenia, który iteracyjnie doskonali politykę agenta.

---


## Przebieg działania

1. **Inicjalizacja:**  
   - Klasa `SodomaAndGomora` dziedziczy po `gym.Env`.
   - Tworzone są mechanizmy synchronizacji (mutexy, eventy) i wątki subskrybujące tematy ROS.
   - Wszystkie wątki (poza głównym) są kontrolowane przez centralny event `state`.

2. **Algorytm PPO:**  
   - Inicjalizacja wewnętrznych modeli PPO.
   - Algorytm iteracyjnie aktualizuje politykę agenta w oparciu o obserwacje.

3. **Pętla symulacji:**
   - Ustawienie wartości sterowania na podstawie modelu PPO.
   - Uruchomienie symulacji i aktywacja wątków przez ustawienie eventu `state`.
   - Po określonym czasie event `state` jest resetowany.
   - Obserwacje są przetwarzane, a model PPO aktualizowany.

---

