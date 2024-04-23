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


def DDA(x0, y0, x1, y1): 
  
    # find absolute differences 
    dx = x1 - x0 
    dy = y1 - y0
  
    # find maximum difference 
    steps = max(abs(dx), abs(dy))
  
    # calculate the increment in x and y 
    dx /= steps
    dy /= steps
  
    # start with 1st point 
    x = float(x0)
    y = float(y0) 
  
    # make a list for coordinates 
    x_coorinates = []
    y_coorinates = []
  
    for _ in range(steps - 1): 
        # append the x,y coordinates in respective list 
        x_coorinates.append(x)
        y_coorinates.append(y)
  
        # increment the values 
        x += dx
        y += dy
    
    return x_coorinates, y_coorinates


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
        # self.map = np.zeros(map_size, dtype=int)
        self.map = 100 * np.ones(map_size, dtype=int)
        # self.map = np.random.rand(*map_size)
        # self.map = 100 * (0.5 < self.map).astype(int)

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

    def update_map(self, x:int, y:int, amount:int) -> None:
        self.map[x,y] += amount
        if self.map[x,y] < 0:
            self.map[x,y] = 0
        elif 100 < self.map[x,y]:
            self.map[x,y] = 100


    def scan_callback(self, scan: LaserScan) -> None:
        """ Take a scan and estimate linear and angular velocity"""

        # Remove points that are inf
        all_ranges = np.array(scan.ranges)

        # non_inf_indices = ~np.isinf(all_ranges)
        
        # ranges = all_ranges[non_inf_indices] # convoluted way of removing rows with -inf or inf
        # angles = scan.angle_min + np.where(non_inf_indices)[0] * scan.angle_increment # get the indices of the ranges that are kept
        ranges = all_ranges
        ranges[np.isinf(ranges)] = 200
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

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
            if (np.zeros(2) < grid_coords).all() and (grid_coords < self.map_size).all():
                robot_grid_pos = self.world_coords2map_coords(self.robot_pose.position.x, self.robot_pose.position.y)
                line_x, line_y = DDA(robot_grid_pos[0], robot_grid_pos[1], grid_coords[0], grid_coords[1])

                self.update_map(grid_coords[0], grid_coords[1], 5)
                # self.map[grid_coords[0], grid_coords[1]] = 

                for x,y in zip(line_x, line_y):
                    if (np.zeros(2) < np.array([x,y])).all() and (np.array([x,y]) < self.map_size).all():
                        self.update_map(int(x), int(y), -5)
                        # self.map[int(x), int(y)] = 
                    # else:
                    #     break
    

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