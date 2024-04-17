import rclpy
from rclpy.node import Node

from scipy.spatial.distance import cdist
import numpy as np

from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point

# Transforms stuff
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# def array2occupancy_grid(map: np.ndarray, map_width: int, map_height: int, map_resolution: float, map_origin: Pose) -> OccupancyGrid:
def array2occupancy_grid(map: np.ndarray):
    return OccupancyGrid(
        header = Header(frame_id = "map"), # the reference frame is the "true" map - which is the top level tf
        info = MapMetaData(
            resolution = 0.05,
            width = 311,
            height = 164,
            origin = Pose(
                position = Point(x = -7.79, y = -4.06, z = 0.0), # TODO: How are we supposed to get this ??
                orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
            )
        ),
        data = map.flatten().astype("int8")
    )


class MapPublisher(Node):

    def __init__(self, map_size: tuple = (164, 311) , publish_frequency: float = 0.5, verbose: bool = False):
        """ map_size: height, width """

        super().__init__('map_publisher')
        self.verbose = verbose

        # Declare and acquire `target_frame` parameter
        # TODO: Dont know why we define it like this
        self.target_frame = self.declare_parameter('target_frame', 'base_link').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State of the robot
        self.robot_position = np.zeros(3)
        self.robot_rotation = np.zeros(4)
        self.robot_pose = Pose()

        # Init map
        self.resolution = 0.05
        self.map_size = np.asarray(map_size)
        self.map = np.zeros(map_size)
        # self.map = np.random.randint(0, 128, size = (map_height, map_width), dtype="int8")
        # threshold = 100
        # self.map[threshold < self.map] = 127
        # self.map[self.map <= threshold] = 0

        # Create a publisher to send map
        self.map_publisher = self.create_publisher(OccupancyGrid, topic = 'map1', qos_profile = 10)
        self.timer = self.create_timer(publish_frequency, self.publish_map)

        # Subscribe to the scan message
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.scan_subscription  # prevent unused variable warning, not sure needed

        # Subscribe to the odometry message
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )
        self.odom_subscription  # prevent unused variable warning, not sure needed


    def world_coords2map_coords(self, x, y) -> np.ndarray:
        return np.floor(np.array([y, x]) / self.resolution + self.map_size / 2.0).astype(int)


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
        self.map_publisher.publish(array2occupancy_grid(self.map))


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
        except TransformException as ex:
            self.get_logger().info(f'Could not transform point in mapping: {ex}')
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
        self.robot_position = np.array([
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z    
        ])
        self.robot_rotation = np.array([
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w
        ])

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