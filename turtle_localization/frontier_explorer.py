import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import numpy as np

from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid


class FrontierPublisher(Node):
    """
    """

    def __init__(self):
        """ """

        super().__init__('frontier_publisher')

        # Subscribe to the map
        self.frontier_subscription = self.create_subscription(
            msg_type = OccupancyGrid,
            topic = 'frontier',
            callback = self.frontier_callback,
            qos_profile = 10
        )

        self.has_goal = False   # Used for setting the first goal
        self.map = None
        self.navigation_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    def send_goal(self):
        pose = self.calculate_new_pose(self.map)
        print(f"Sending new goal: {pose.pose.pose.position.x},{pose.pose.pose.position.y}")

        self.navigation_action_client.wait_for_server()

        self._send_goal_future = self.navigation_action_client.send_goal_async(pose)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected')
            return
        
        print('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        """ Called once the goal has finished """
        result = future.result().result
        print('Result: {0}'.format(result))

        # Send a new random goal
        self.send_goal()


    def coord2pose(self, coords: np.ndarray) -> Pose:
        """ Turn a 2D coord into a Pose - could be a better function"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = float(coords[0])
        goal_msg.pose.pose.position.y = float(coords[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 0.0
        goal_msg.pose.header.frame_id = "map"   # The world coordinate frame
        return goal_msg


    def map2world_coords(self, grid_x, grid_y) -> np.ndarray:
        # TODO: There is a bug here i am almost certain - should be fixable tho
        return (np.array([grid_x, grid_y]) - np.array([self.map_width, self.map_height]) / 2.0) * self.map_resolution

    
    def select_random_grid_coord(self, frontier_grid: np.ndarray) -> tuple:
        """ Select a random 2D coordinate from a boolean grid """
        indices = np.where(frontier_grid)
        random_index = np.random.choice(np.arange(len(indices)), size=1)[0]
        row = indices[1][random_index]
        col = indices[0][random_index]
        return row, col


    def calculate_new_pose(self, frontier_grid: np.ndarray) -> Pose:
        # Could be any method, here we select a random grid coord
        row, col = self.select_random_grid_coord(frontier_grid=frontier_grid)

        # Convert from grid coordinate to world coordinate
        world_coords = self.map2world_coords(col, row)

        return self.coord2pose(world_coords)


    def frontier_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """ Take a frontier occupancy grid and give a next goal pose """
    
        self.map_width = occupancy_grid.info.width   # 155
        self.map_height = occupancy_grid.info.height # 82 
        self.map_resolution = occupancy_grid.info.resolution # 0.1

        self.map = np.array(occupancy_grid.data).reshape(self.map_height, self.map_width)
        
        # First time we get map info we set the goal of the robot
        if not self.has_goal:
            self.send_goal()
            self.has_goal = True


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