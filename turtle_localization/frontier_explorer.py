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


class FrontierPublisher(Node):
    """
    """

    def __init__(self, publish_frequncy:float = 0.1):
        """ map_size: height, width """

        super().__init__('frontier_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # This has to be set although it isn't used
    
        # Subscribe to the map
        self.frontier_subscription = self.create_subscription(
            msg_type = OccupancyGrid,
            topic = 'frontier',
            callback = self.frontier_callback,
            qos_profile = 10
        )

        self.navigation_goal_publisher = self.create_publisher(
            OccupancyGrid, 
            topic = 'frontier', 
            qos_profile = 10
        )
        self.timer = self.create_timer(publish_frequncy, self.publish_navigation_goal)


    def publish_navigation_goal(self):
        """ """
        if self.goal is not None:
            self.navigation_goal_publisher.publish()# TODO


    def calculate_new_pose():
        pass


    def frontier_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """ Take a frontier occupancy grid and give a next goal pose """

        map_width = occupancy_grid.info.width
        map_height = occupancy_grid.info.height

        map = np.array(occupancy_grid.data).reshape(map_height, map_width)

        # Get the transform from robot to world
        try:
            robot2world_transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time().to_msg())
        except TransformException as exception:
            self.get_logger().info(f'Could not transform point in mapping: {exception}')
            return

        

        pass


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