from enum import IntEnum
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import numpy as np

from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from tf2_geometry_msgs import do_transform_point

# Transforms stuff
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrontierPublisher(Node):
    """
    """

    def __init__(self, publish_frequncy:float = 0.1):
        """ map_size: height, width """

        super().__init__('frontier_publisher')

        # Subscribe to the map
        # self.frontier_subscription = self.create_subscription(
        #     msg_type = OccupancyGrid,
        #     topic = 'frontier',
        #     callback = self.frontier_callback,
        #     qos_profile = 10
        # )

        # self.navigation_goal_publisher = self.create_publisher(
        #     OccupancyGrid, 
        #     topic = 'frontier', 
        #     qos_profile = 10
        # )
        # self.timer = self.create_timer(publish_frequncy, self.publish_navigation_goal)
        self.navigation_action_client = ActionClient(self, PoseStamped, 'navigate_to_pose')


    def send_goal(self, pose):
        pose.header.frame_id = "map"
        # self.coord2pose(np.array([0.0, 0.0]))
        self.navigation_action_client.wait_for_server()
        return self.navigation_action_client.send_goal_async(pose)


    # def publish_navigation_goal(self):
    #     """ """
    #     if self.goal is not None:
    #         self.navigation_goal_publisher.publish()# TODO


    # def coord2pose(self, coords: np.ndarray) -> Pose:
    #     """ Turn a 2D coord into a Pose - could be a better function"""
    #     return PoseStamped(
    #         header = Header(
    #         ),
    #         pose = Pose(
    #             position = Point(
    #                 x = coords[0],
    #                 y = coords[1],
    #                 z = 0.0,
    #         ),
    #             orientation = Quaternion(
    #                 x = 0.0,
    #                 y = 0.0,
    #                 z = 0.0,
    #                 w = 0.0,
    #             )
    #         )
    #     )

    # def map2world_coords(self, x:int, y:int) -> np.ndarray:
    #     """
    #     """
    #     # map 2 world
    #     # return np.array([y, x]) * self.map_resolution + self.map_size / 2.0
    #     return np.array([y, x]) * 0.1 + np.array([82, 155]) / 2.0

    
    # def select_random_grid_coord(self, frontier_grid: np.ndarray) -> tuple:
    #     """ Select a random 2D coordinate from a boolean grid """
    #     indices = np.where(frontier_grid)
    #     random_index = np.random.choice(np.arange(len(indices)), size=1)
    #     row = indices[1][random_index]
    #     col = indices[0][random_index]
    #     return row, col


    # def calculate_new_pose(self, frontier_grid: np.ndarray) -> Pose:
    #     # Could be any method, here we select a random grid coord
    #     row, col = self.select_random_grid_coord(frontier_grid=frontier_grid)

    #     # Convert from grid coordinate to world coordinate
    #     world_coords = self.map2world_coords(col, row)

    #     return self.coord2pose(world_coords) # TODO: 


    # def frontier_callback(self, occupancy_grid: OccupancyGrid) -> None:
    #     """ Take a frontier occupancy grid and give a next goal pose """

    #     map_width = occupancy_grid.info.width
    #     map_height = occupancy_grid.info.height

    #     map = np.array(occupancy_grid.data).reshape(map_height, map_width)

    #     # pose = self.calculate_new_pose(map)

    #     # if not self.has_sent:
    #     #     print("Sending goal")
    #     #     self.send_goal()
    #     #     self.has_sent = True


def main(args=None):
    rclpy.init(args=args)

    frontier_pub = FrontierPublisher()
    future = frontier_pub.send_goal(PoseStamped())

    rclpy.spin_until_future_complete(frontier_pub, future)
    # rclpy.spin(frontier_pub)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # frontier_pub.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()