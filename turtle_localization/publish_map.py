import rclpy
from rclpy.node import Node

from scipy.spatial.distance import cdist
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


class MapPublisher(Node):
    """ """

    def __init__(self, map_size: tuple = (164, 311), map_resolution: float = 0.05, publish_frequency: float = 0.5, verbose: bool = False):
        """ map_size: height, width """

        super().__init__('map_publisher')
        self.verbose = verbose
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # This has to be set although it isn't used

        # State of the robot
        self.robot_pose = Pose()

        # Init map
        self.map_resolution = map_resolution
        self.map_size = np.array(map_size, dtype=int)
        self.map = np.zeros(map_size, dtype=int)

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


    def array2occupancy_grid(self, map: np.ndarray):
        return OccupancyGrid(
            header = Header(frame_id = "map"), # the reference frame is the "true" map - which is the top level tf
            info = MapMetaData(
                resolution = self.map_resolution,
                width = int(self.map_size[1]), # TODO: Dont know why we have to cast here, shouldnt be neccesary
                height = int(self.map_size[0]), # TODO: Dont know why we have to cast here, shouldnt be neccesary
                origin = Pose(
                    position = Point(x = -7.79, y = -4.06, z = 0.0), # TODO: How are we supposed to get this ??
                    orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
                )
            ),
            data = map.flatten().astype("int8")
        )


    def world_coords2map_coords(self, x, y) -> np.ndarray:
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

        # Remove points that are inf
        all_ranges = np.array(scan.ranges)

        non_inf_indices = ~np.isinf(all_ranges)
        
        ranges = all_ranges[non_inf_indices] # convoluted way of removing rows with -inf or inf
        angles = scan.angle_min + np.where(non_inf_indices)[0] * scan.angle_increment # get the indices of the ranges that are kept

        # Update the target and current points
        points = self.compute_world_coords(ranges, angles)

        # Get the transform from robot to world
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time().to_msg())
        except TransformException as exception:
            if self.verbose:
                self.get_logger().info(f'Could not transform point in mapping: {exception}')
            return

        # Plot all the points
        for point in points:
            point = PointStamped(
                header = Header(frame_id = "base_link"),
                point = Point(x = point[0], y = point[1], z = point[2])
            )
            t = do_transform_point(point, transform)

            grid_coords = self.world_coords2map_coords(t.point.x, t.point.y)

            # Write the point to the grid
            if (grid_coords < self.map_size).all():
                self.map[grid_coords[0], grid_coords[1]] = 100
    

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