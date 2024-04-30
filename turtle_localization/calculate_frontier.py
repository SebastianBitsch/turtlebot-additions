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


def bfs(grid: np.ndarray, start: tuple) -> np.ndarray:
    """ Simple BFS implementation on a grid """

    h, w = grid.shape
    visited = np.zeros_like(grid, dtype=bool)   # Keep track of which cells we have been to
    frontier = np.zeros_like(grid, dtype=bool)  # The frontier map to publish
    queue = deque()
 
    queue.append(start)
    visited[*start] = True
    
    while 0 < len(queue):
        cell = queue.popleft()
        x = cell[0]
        y = cell[1]
  
        # Check neighbours
        for dx, dy in [(-1,0), (0,1), (1,0), (0,-1)]: # The four neighbour directions; N,E,S,W 
            row = x + dx
            col = y + dy
            
            # Check out of bounds
            if row < 0 or col < 0 or h <= row or w <= col:
                continue

            # Check if already been here
            if visited[row, col]:
                continue

            # Check if wall
            if 80 < grid[row, col]:
                continue

            # Check if neighbour is unknown, if so we found a frontier
            if grid[row, col] == -100: # TODO: -100 is our UNEXPLORED CELL TYPE, shouldn't be hardcoded
                frontier[x,y] = True
                continue
        
            queue.append((row, col))
            visited[row][col] = True

    return frontier


class FrontierPublisher(Node):
    """
    """

    def __init__(self, publish_frequncy:float = 0.1):
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
        self.timer = self.create_timer(publish_frequncy, self.publish_frontier)


    def array2occupancy_grid(self, map: np.ndarray):
        """ Convert an array to OccupancyGrid message type. See: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html"""
        # TODO: The same function is also in publish_map.py - should create a commons.py to have these functions
        return OccupancyGrid(
            header = Header(frame_id = "map"), # the reference frame is the "true" map - which is the top level tf
            info = MapMetaData(
                resolution = 0.1, # TODO: Same as the other map, shouldnt be hardcoded
                width = map.shape[1],
                height = map.shape[0],
                origin = Pose(
                    position = Point(x = -7.79, y = -4.06, z = 0.0), # TODO: This is the hardcoded offset to make the map line up perfectly with the existing map, not nescessary 
                    orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
                )
            ),
            data = 100 * map.flatten().astype("int8")
        )

    def publish_frontier(self):
        """ """
        if self.map is not None:
            self.frontier_publisher.publish(self.array2occupancy_grid(self.map))


    def map_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """ Take a scan and estimate linear and angular velocity"""
        
        map_width = occupancy_grid.info.width
        map_height = occupancy_grid.info.height

        map = np.array(occupancy_grid.data).reshape(map_height, map_width)

        start = (map_height // 2, map_width // 2) # TODO: We cant assume that the map center will always be explored - we could start from a corner etc.

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