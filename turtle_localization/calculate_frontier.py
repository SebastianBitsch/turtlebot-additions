from enum import IntEnum
from collections import deque

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from tf2_geometry_msgs import do_transform_point

# Transforms stuff
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



# Direction vectors
dRow = [ -1, 0, 1, 0]
dCol = [ 0, 1, 0, -1]
 
# Function to check if a cell
# is be visited or not
def isValid(vis, grid, row, col):
   
    # If cell lies out of bounds
    if (row < 0 or col < 0 or row >= 82 - 1 or col >= 155 - 1):
        return False
    
    if grid[row][col] == -100:
        return False
 
    # If cell is already visited
    if (vis[row][col]):
        return False

    # Otherwise
    return True
 
# Function to perform the BFS traversal
def bfs(grid: np.ndarray, start: tuple) -> np.ndarray:
    
    visited = np.zeros_like(grid, dtype=bool)
    frontier = np.zeros_like(grid, dtype=bool)
    queue = deque()
 
    queue.append((start[0], start[1]))
    visited[*start] = True
 
    while 0 < len(queue):
        cell = queue.popleft()
        x = cell[0]
        y = cell[1]
  
        # Check neighbours
        for i in range(4):
            row = x + dRow[i]
            col = y + dCol[i]
            
            # Check out of bounds
            if row < 0 or col < 0 or 82 <= row or 155 <= col:
                continue

            # Check if already been here
            if visited[row, col]:
                continue

            # Check if wall
            if 80 < grid[row, col]:
                continue

            # Check if neighbour is unknown, if so we found a frontier
            if grid[row, col] == -100:
                frontier[x,y] = True
                continue
        
            queue.append((row, col))
            visited[row][col] = True

    return frontier


class FrontierPublisher(Node):
    """
    """

    def __init__(self):
        """ map_size: height, width """

        super().__init__('frontier_publisher')

        self.map = None
    
        # Subscribe to the map
        self.map_subscription = self.create_subscription(
            msg_type = OccupancyGrid,
            topic = 'map1',
            callback = self.map_callback,
            qos_profile = 10
        )

        self.frontier_publisher = self.create_publisher(
            OccupancyGrid, 
            topic = 'frontier', 
            qos_profile = 10
        )
        self.timer = self.create_timer(2.0, self.publish_frontier)


    def array2occupancy_grid(self, map: np.ndarray):
        return OccupancyGrid(
            header = Header(frame_id = "map"), # the reference frame is the "true" map - which is the top level tf
            info = MapMetaData(
                resolution = 0.1,
                width = 155, # TODO: Dont know why we have to cast here, shouldnt be neccesary
                height = 82, # TODO: Dont know why we have to cast here, shouldnt be neccesary
                origin = Pose(
                    position = Point(x = -7.79, y = -4.06, z = 0.0), # TODO: This is the hardcoded offset to make the map line up perfectly with the existing map, not nescessary 
                    orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
                )
            ),
            data = 100 * map.flatten().astype("int8")
        )

    def publish_frontier(self):
        """ """
        print("pub", np.sum(self.map), self.map.shape)
        self.frontier_publisher.publish(self.array2occupancy_grid(self.map))


    def map_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """ Take a scan and estimate linear and angular velocity"""
        
        map_width = occupancy_grid.info.width
        map_height = occupancy_grid.info.height

        map = np.array(occupancy_grid.data).reshape(map_height, map_width)

        start = (map_height // 2, map_width // 2)

        self.map = bfs(map, start)
        


        



def main(args=None):
    rclpy.init(args=args)

    frontier_pub = FrontierPublisher()

    rclpy.spin(frontier_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frontier_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()