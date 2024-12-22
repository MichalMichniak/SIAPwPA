import numpy as np
import matplotlib.pyplot as plt
import csv

data = []
# with open('/home/developer/ros2_ws/src/scripts/wewnetrzne_git.csv', 'r') as file:
# with open('/home/developer/ros2_ws/src/scripts/zewnetrzne_git.csv', 'r') as file:
with open('/home/developer/ros2_ws/src/scripts/srodkowe.csv', 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)
    for row in csv_reader:
        data.append([float(row[0]), float(row[1])])

def oblicz_odleglosc(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

odleglosci = []

for i in range(1, len(data)):
    punkt1 = data[i-1]
    punkt2 = data[i]
    odleglosci.append(oblicz_odleglosc(punkt1, punkt2))

plt.figure(figsize=(10, 6))
plt.plot(range(1, len(odleglosci) + 1), odleglosci, marker='o', linestyle='-', color='b')
plt.title('Odległość między kolejnymi punktami')
plt.xlabel('Numer próbki')
plt.ylabel('Odległość')
plt.grid(True)
plt.show()