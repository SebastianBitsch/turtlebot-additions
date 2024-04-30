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


def DDA(a:np.ndarray, b:np.ndarray) -> np.ndarray:
    """ Digital differential analyzer. See: https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm) """
    delta = (b - a).astype(float)
    steps = np.max(np.abs(delta)).astype(int)
    delta /= steps
    return [a + i * delta for i in range(steps)]


class MapPublisher(Node):
    """
    """

    def __init__(self, map_size: tuple = (164, 311), map_resolution: float = 0.1, publish_frequency: float = 0.5):
        """ map_size: height, width """

        super().__init__('map_publisher')
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # This has to be set although it isn't used

        # State of the robot
        self.robot_pose = Pose()

        # Init map
        self.map_resolution = map_resolution
        self.map_size = np.array(map_size, dtype=int) // 2
        self.map = 50 * np.ones(self.map_size, dtype=int)

        # Create a publisher to send map
        self.map_publisher = self.create_publisher(OccupancyGrid, topic = 'map1', qos_profile = 10)
        self.timer = self.create_timer(publish_frequency, self.publish_map)

        # Subscribe to the scan message
        self.scan_subscription = self.create_subscription(
            msg_type = LaserScan,
            topic = 'scan',
            callback = self.scan_callback,
            qos_profile = 10
        )

        # Subscribe to the odometry message
        self.odom_subscription = self.create_subscription(
            msg_type = Odometry,
            topic = 'odom',
            callback = self.odometry_callback,
            qos_profile = 10
        )


    def is_point_on_map(self, point: np.ndarray) -> bool:
        assert point.shape == (2,), f"Error: Can only check if 2D points are on map. Point had shape {point.shape}"
        return (np.zeros(2) < point).all() and (point < self.map_size).all()


    def array2occupancy_grid(self, map: np.ndarray):
        return OccupancyGrid(
            header = Header(frame_id = "map"), # the reference frame is the "true" map - which is the top level tf
            info = MapMetaData(
                resolution = self.map_resolution,
                width = int(self.map_size[1]), # TODO: Dont know why we have to cast here, shouldnt be neccesary
                height = int(self.map_size[0]), # TODO: Dont know why we have to cast here, shouldnt be neccesary
                origin = Pose(
                    position = Point(x = -7.79, y = -4.06, z = 0.0), # TODO: How are we supposed to get this ?? This is the center of the map
                    orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
                )
            ),
            data = map.flatten().astype("int8")
        )


    def world2map_coords(self, x, y) -> np.ndarray:
        """
        """
        return np.floor(np.array([y, x]) / self.map_resolution + self.map_size / 2.0).astype(int)


    def compute_world_coords(self, ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
        """ Assumes 2D scanner, i.e. (probably) wont work in 3D """
        # angles = np.arange(scan.angle_min, scan.angle_max, step = scan.angle_increment)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros(len(ranges))

        coords = np.vstack([x, y, z]).T # [360, 3] for instance
        return coords
    

    def publish_map(self):
        """ """
        self.map_publisher.publish(self.array2occupancy_grid(self.map))


    def scan_callback(self, scan: LaserScan) -> None:
        """ Take a scan and estimate linear and angular velocity"""

        ranges = np.array(scan.ranges)

        # Replace angles where the scanner didn't hit (inf) with the max range of the scanner
        hits = ~np.isinf(ranges)
        ranges[~hits] = scan.range_max
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        # Calculate the end points of the 360 scan lines
        end_points = self.compute_world_coords(ranges, angles)

        # Get the transform from robot to world
        try:
            robot2world_transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time().to_msg())
        except TransformException as exception:
            self.get_logger().info(f'Could not transform point in mapping: {exception}')
            return

        # Plot all the points
        for end_point, hit in zip(end_points, hits):
            end_point = PointStamped(
                header = Header(frame_id = "base_link"),
                point = Point(x = end_point[0], y = end_point[1], z = end_point[2])
            )

            # Convert the endpoint of the ray to a coordinate on the map
            end_point = do_transform_point(end_point, robot2world_transform)
            grid_coords = self.world2map_coords(end_point.point.x, end_point.point.y)

            # Write the point to the grid
            if self.is_point_on_map(grid_coords):
                robot_grid_pos = self.world2map_coords(self.robot_pose.position.x, self.robot_pose.position.y)

                # Generate the line from the robot to the end of the raycast line
                line_points = DDA(robot_grid_pos, grid_coords)

                # Populate the map in the cells where the ray intersects
                for x, y in line_points:
                    if self.is_point_on_map(np.array([x,y])):
                        if 0 < self.map[int(x), int(y)]:
                            self.map[int(x), int(y)] -= 1

                # Handle direct hits to the objects of the ray cast
                if hit:
                    if self.map[grid_coords[0], grid_coords[1]] < 95:
                        self.map[grid_coords[0], grid_coords[1]] += 5


    def odometry_callback(self, odometry: Odometry) -> None:
        """ Update the robot position and rotation from the odometry """
        self.robot_pose = odometry.pose.pose



def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()