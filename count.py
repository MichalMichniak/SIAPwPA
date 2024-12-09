import pandas as pd
import numpy as np
wew = pd.read_csv("po_wewnetrznej_1.csv")
zew = pd.read_csv("po_zewnetrznej1.csv")

class GetDistances:
    def __init__(self, wew, zew, grids=20):
        self.zew = zew
        self.wew = wew
        self.grids = grids
        self.x_sizes = (min(zew["x"]), max(zew["x"]))
        self.y_sizes = (min(zew["y"]), max(zew["y"]))
        self.x_step = (self.x_sizes[1] - self.x_sizes[0]) / grids
        self.y_step = (self.y_sizes[1] - self.y_sizes[0]) / grids
        self.mapped_data_in = self.create_grids(self.wew)
        self.mapped_data_out = self.create_grids(self.zew)

# ADD DATA TO PROPER GRIDS
    def create_grids(self, df):
        mapped_data = {(i, j): [] for i in range(self.grids) for j in range(self.grids)}
        for _, row in df.iterrows():
            mapped_data[self.find_grid_nums(int(row['x']), int(row['y']))].append((row['x'], row['y']))
        return mapped_data

    def find_grid_nums(self, x_pos, y_pos):
        x_grid_num = (x_pos - self.x_sizes[0]) // self.x_step
        y_grid_num = (y_pos - self.y_sizes[0]) // self.y_step
        return (x_grid_num, y_grid_num)

    def get_distances(self, x, y):
        distance_in = np.inf
        grid_i, grid_j = self.find_grid_nums(x, y)
        for data in self.mapped_data_in[(grid_i, grid_j)]:
            dist = np.linalg.norm(np.array((x,y)) - np.array(data))
            if dist < distance_in: 
                distance_in = dist
        distance_out = np.inf
        for data in self.mapped_data_out[(grid_i, grid_j)]:
            dist = np.linalg.norm(np.array((x,y)) - np.array(data))
            if dist < distance_out: 
                distance_out = dist
        return distance_in, distance_out

find_len = GetDistances(wew, zew, 20)
# print(find_len.x_sizes)
# print(find_len.y_sizes)
# print(find_len.x_step)
# print(find_len.y_step)

# # print(find_len.find_grid_nums(400, -200))
# print(len(find_len.mapped_data_in[(0,0)]))
# print(len(find_len.mapped_data_in[(0,1)]))
# print(len(find_len.mapped_data_in[(1,0)]))
# print(len(find_len.mapped_data_in[(1,1)]))

print(find_len.get_distances(-343,426))
# print(find_len.mapped_data_in)