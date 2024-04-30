from enum import IntEnum
from collections import deque as queue

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
def isValid(vis, row, col):
   
    # If cell lies out of bounds
    if (row < 0 or col < 0 or row >= 82 - 1 or col >= 155 - 1):
        return False
 
    # If cell is already visited
    if (vis[row][col]):
        return False

    # Otherwise
    return True
 
# Function to perform the BFS traversal
def BFS(grid, vis, row, col):
   
    # Stores indices of the matrix cells
    q = queue()
 
    # Mark the starting cell as visited
    # and push it into the queue
    q.append(( row, col ))
    vis[row][col] = True
 
    # Iterate while the queue
    # is not empty
    while (0 < len(q)):
        cell = q.popleft()
        x = cell[0]
        y = cell[1]
        print(grid[x][y], end = " ")
 
        #q.pop()
 
        # Go to the adjacent cells
        for i in range(4):
            adjx = x + dRow[i]
            adjy = y + dCol[i]
            if (isValid(vis, adjx, adjy)):
                q.append((adjx, adjy))
                vis[adjx][adjy] = True
 



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


    def publish_map(self):
        """ """
        self.frontier_publisher.publish(self.array2occupancy_grid(self.map))


    def map_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """ Take a scan and estimate linear and angular velocity"""
        
        map_width = occupancy_grid.info.width
        map_height = occupancy_grid.info.height

        self.map = np.array(occupancy_grid.data).reshape(map_height, map_width)

        start = (map_height // 2, map_width // 2)
        print(self.map.shape, start)
        print("------")
        print(BFS(self.map, np.zeros_like(self.map), *start))
        


        



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