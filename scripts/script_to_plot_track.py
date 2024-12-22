import matplotlib.pyplot as plt
import csv

def plot_csv_data(file1, file2, file3):
    def read_csv(file):
        x, y = [], []
        with open(file, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                x.append(float(row[0]))
                y.append(float(row[1]))
        return x, y

    x1, y1 = read_csv(file1)
    x2, y2 = read_csv(file2)
    x3, y3 = read_csv(file3)

    plt.figure(figsize=(10, 6))
    plt.scatter(x1, y1, color='blue', label='wewnetrzne', alpha=0.7, s=0.1)
    plt.scatter(x2, y2, color='red', label='zewnetrzne', alpha=0.7, s=0.1)
    plt.scatter(x3, y3, color='green', label='srodkowe', alpha=0.7, s=0.1)

    plt.title('Mapa')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    plt.grid(True)
    plt.show()


file1 = '/home/developer/ros2_ws/src/scripts/wewnetrzne_git.csv'
file2 = '/home/developer/ros2_ws/src/scripts/zewnetrzne_git.csv'
file3 = '/home/developer/ros2_ws/src/scripts/srodkowe.csv'

plot_csv_data(file1, file2, file3)