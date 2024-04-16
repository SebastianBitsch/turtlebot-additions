import rclpy
from rclpy.node import Node

from scipy.spatial.distance import cdist
import numpy as np

from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData


def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix


# def array2occupancy_grid(map: np.ndarray, map_width: int, map_height: int, map_resolution: float, map_origin: Pose) -> OccupancyGrid:
def array2occupancy_grid(map: np.ndarray):
    return OccupancyGrid(
        header = Header(frame_id = "map"), # this is the "true" map - dont know if thats correct
        info = MapMetaData(
            resolution = 0.05,
            width = 311,
            height = 164,
            origin = Pose(
                position = Point(x = -7.79, y = -4.06, z = 0.0),
                orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
            )
        ),
        data = map.flatten().astype("int8")
    )


class MapPublisher(Node):

    def __init__(self, map_width: int = 311, map_height: int = 164, publish_frequency: float = 0.5, verbose: bool = False):
        super().__init__('map_publisher')
        self.verbose = verbose

        # State of the robot
        self.robot_position = np.zeros(3)
        self.robot_rotation = np.zeros(4)

        # Init map
        self.width = map_width
        self.height = map_height
        self.map_size = np.array([map_height, map_width])
        self.resolution = 0.05
        self.map = np.zeros((map_height, map_width))
        # self.map = np.random.randint(0, 128, size = (map_height, map_width), dtype="int8")
        # threshold = 100
        # self.map[threshold < self.map] = 127
        # self.map[self.map <= threshold] = 0

        # Create a publisher to send map
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map1', 10) # qos = 10
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
        return np.floor(np.array([x,y]) / self.resolution + self.map_size / 2).astype("uint8")

    def transform(self, x: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """ rotate and move a vector, not sure transform is the right word """
        assert R.shape == (3,3) and t.shape == (3,) and x.shape[1] == 3, "Wrong shapes"
        return (R @ x.T + t[:, None]).T

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

        R = quaternion_rotation_matrix(self.robot_rotation)
        
        # world_coords = self.robot_position + points @ np.linalg.inv(R)
        # print(world_coords[0])
        


    def odometry_callback(self, odometry: Odometry) -> None:
        """ Update the robot position and rotation from the odometry """
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
        grid_coords = self.world_coords2map_coords(self.robot_position[1], self.robot_position[0])
        print(grid_coords, self.map.shape)
        self.map[grid_coords[0], grid_coords[1]] = 100
        # print(self.map.shape)



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