import pandas as pd
import numpy as np

class GetDistances:
    def __init__(self, wew, zew, grid_size=20):
        self.zew = zew
        self.wew = wew
        self.grid_size = grid_size
        self.x_sizes = (min(zew["x"]), max(zew["x"]))
        self.y_sizes = (min(zew["y"]), max(zew["y"]))
        self.x_step = (self.x_sizes[1] - self.x_sizes[0]) / grid_size
        self.y_step = (self.y_sizes[1] - self.y_sizes[0]) / grid_size
        self.mapped_data_in = self.create_grids(self.wew)
        self.mapped_data_out = self.create_grids(self.zew)

# ADD DATA TO PROPER GRIDS
    def create_grids(self, df):
        mapped_data = [[[] for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        for _, row in df.iterrows():
            grid_idxs = self.find_grid_nums(row['x'], row['y'])
            mapped_data[grid_idxs[0]][grid_idxs[1]].append((row['x'], row['y']))
        return mapped_data

    def find_grid_nums(self, x_pos, y_pos):
        x_grid_num = (x_pos - self.x_sizes[0]) // self.x_step
        y_grid_num = (y_pos - self.y_sizes[0]) // self.y_step
        return min(int(x_grid_num), self.grid_size-1), min(int(y_grid_num), self.grid_size-1)
    
    def get_rewards(self, x, y):
        in_data = self.get_distance(x, y, self.mapped_data_in)
        out_data = self.get_distance(x, y, self.mapped_data_out)
        print(in_data, out_data)
        dist_between = np.hypot(out_data[1][0] - in_data[1][0], out_data[1][1] - in_data[1][1])
        return in_data[0], out_data[0], dist_between

    def get_distance(self, x, y, grid):
        distance = np.inf
        grid_i, grid_j = self.find_grid_nums(x, y)
        point = None
        i_min = max(0, grid_i - 1)
        i_max = min(self.grid_size - 1, grid_i + 1)
        j_min = max(0, grid_j - 1)
        j_max = min(self.grid_size - 1, grid_j + 1)
        point = [0,0]
        for idx1 in range(i_min, i_max + 1):
            for idx2 in range(j_min, j_max + 1):
                for data in grid[idx1][idx2]:
                    dist = np.hypot(x - data[0], y - data[1]) #hypot to linalg.norm ale podobno szybsze
                    if dist < distance:
                        distance = dist
                        point = data
                        
        return distance, point