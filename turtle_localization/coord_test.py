import numpy as np


map_resolution = 0.1
map_size = np.array((164, 311), dtype=int) // 2
map_width = int(map_size[1])
map_height = int(map_size[0])


def map2world_coords(grid_x, grid_y) -> np.ndarray:
    # x = (grid_x - (map_height/2)) * map_resolution
    # y = (grid_y - (map_width/2)) * map_resolution
    # return np.array([x, y])
    #return (np.array([grid_x, grid_y]) - (np.array([map_width, map_height]) / 2.0)) * map_resolution # This doesnt work
    return (np.array([grid_x, grid_y]) - (np.array([map_height, map_width]) / 2.0)) * map_resolution # This works :D


def world2map_coords(x, y) -> np.ndarray:
        """
        """
        return np.floor(np.array([x, y]) / map_resolution + map_size / 2.0).astype(int)


x, y = 1.5135673372597425,0.07111306065369605
#x,y = 0.0, 0.0
print("Starting with [x,y]: [{},{}]".format(x,y))
m2w = world2map_coords(x, y)
print("Grid indicies:", m2w)

w2m = map2world_coords(m2w[0], m2w[1])
print("Coords from indicies:", w2m)





