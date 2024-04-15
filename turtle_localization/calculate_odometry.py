import rclpy
from rclpy.node import Node

from scipy.spatial.distance import cdist
import numpy as np
from sklearn.neighbors import NearestNeighbors


from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from std_msgs.msg import Header


def calculate_velocities(R_prev, t_prev, R_current, t_current, delta_t):
    # Calculate change in translation over delta_t
    delta_translation = (t_current - t_prev) / delta_t
    
    # Calculate change in rotation over delta_t
    delta_rotation = np.dot(R_current, R_prev.T)
    
    # Convert delta_rotation to axis-angle representation
    angle = np.arccos((np.trace(delta_rotation) - 1) / 2)
    axis = 1 / (2 * np.sin(angle)) * np.array([delta_rotation[2, 1] - delta_rotation[1, 2],
                                                delta_rotation[0, 2] - delta_rotation[2, 0],
                                                delta_rotation[1, 0] - delta_rotation[0, 1]])
    
    # Calculate angular velocity (in radians per second)
    angular_velocity = axis * angle / delta_t
    
    # Linear velocity is the change in translation over delta_t
    linear_velocity = delta_translation / delta_t
    
    return linear_velocity, angular_velocity


def point_cloud(points, parent_frame: str):

    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = Header(frame_id=parent_frame)

    return PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]), 
        data=data
    )

class CalculateOdometry(Node):

    def __init__(self, verbose: bool = False):
        super().__init__('calculate_odometry')

        # time series of rotation matrices and transformation vectors
        self.Rs = [np.eye(3)]
        self.ts = [np.zeros(3)]
        self.x = None
        self.y = None

        self.points = np.random.randn(10,3) # init to random points
        self.pcd_publisher = self.create_publisher(PointCloud2, 'pcd', 10)
        timer_period = 1/30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.verbose = verbose

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning, not sure needed

    def timer_callback(self):
        self.pcd = point_cloud(self.points, 'base_footprint') # the baselink of the robot
        self.pcd_publisher.publish(self.pcd)

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
    
    def weighted_average(self, values: np.ndarray, weights: np.ndarray = None) -> np.ndarray:
        """ Get the centre of mass for a set of values, """
        return np.average(values, weights=weights, axis=0)

    def compute_transformation_params(self, x: np.ndarray, y: np.ndarray, p: np.ndarray = None) -> tuple[np.ndarray, np.ndarray]:
        """
        Perform a single step of ICP where perfect corresponence is assumed. 
        Returns a rotation matrix and translation vector.
        """
        assert x.shape == y.shape
        if p is None:
            p = np.ones(len(x))
        # Get the centres of mass
        x_0 = self.weighted_average(x, p)
        y_0 = self.weighted_average(y, p)

        # Calculate cross covariance matrix
        H = (y_0 - y).T @ ((x_0 - x) * p[:, None])

        # Do SVD
        U, _S, Vh = np.linalg.svd(H)    

        # Calculate rotation matrix and translation vector
        R = Vh.T @ U.T
        t = y_0 - R @ x_0

        return R, t
    
    def compute_closest_pairs(self, x: np.ndarray, y: np.ndarray, distances: np.ndarray, indices: np.ndarray, threshold: float) -> tuple[np.ndarray, np.ndarray]:
        """ """
        closest_point_pairs = []
        for nn_index in range(len(distances)):
            if distances[nn_index][0] < threshold:
                closest_point_pairs.append((x[nn_index], y[indices[nn_index][0]]))

        closest_point_pairs = np.array(closest_point_pairs)
        return closest_point_pairs[:,0,:], closest_point_pairs[:,1,:]
    

    def ICP(self, x: np.ndarray, y: np.ndarray, max_iters: int = 100) -> tuple[np.ndarray, np.ndarray]:
        """
        Does vanilla point-to-point ICP. See: https://en.wikipedia.org/wiki/Iterative_closest_point

        Args:
            x (np.ndarray): Source point cloud, shape (N, 3).
            y (np.ndarray): Target point cloud, shape (M, 3).
            max_iters (int, optional): Maximum number of iterations. Defaults to 100.

        Returns:
            tuple[np.ndarray, np.ndarray]: Tuple containing the rotation matrix (R) and translation vector (t) to align x to y.
        """
        R = np.eye(3)
        t = np.zeros(3)

        # Safe-guard against (some) dumb input
        if np.array_equal(x, y):
            return R, t

        prev_indices = None

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(y)

        for i in range(max_iters):
    
            x_hat = self.transform(x, R, t)

            # Get matching
            distances, indices = nbrs.kneighbors(x_hat)
            distances = distances.ravel()
            indices = indices.ravel()
            
            # Find outliers
            inlier_indices = distances < np.mean(distances) + 2 * np.std(distances)
            indices = indices[inlier_indices]

            # Remove outliers
            x_hat = x_hat[inlier_indices]
            y_hat = y[indices]

            # Fit points
            R_delta, t_delta = self.compute_transformation_params(x_hat, y_hat)
            R = R @ R_delta
            t = t + t_delta

            # Calculate error
            if self.verbose:
                mse = np.mean((x_hat - y_hat)**2)
                print(f"ICP error @ iteration {i + 1}: {mse:.5f} | n points {len(x_hat)}")

            # If nothing is changed, we stop
            if np.array_equal(indices, prev_indices):
                break

            prev_indices = indices

        return R, t


    def log_rotation(self, R: np.ndarray) -> np.ndarray:
        """
        Extracts the axis-angle representation of a rotation matrix.
        
        Parameters:
            R (np.ndarray): Rotation matrix of shape (3, 3).
            
        Returns:
            np.ndarray: Axis-angle representation of shape (3,).
        """
        # Compute the trace of the rotation matrix
        trace = np.trace(R)
        
        # Calculate the angle of rotation
        angle = np.arccos((trace - 1) / 2)
        
        # Calculate the axis of rotation
        axis = (R - R.T) / (2 * np.sin(angle))
        
        # Ensure that the axis is a unit vector
        axis_norm = np.linalg.norm(axis)
        if axis_norm != 0:
            axis /= axis_norm
        
        # Scale the axis by the angle to get the axis-angle representation
        axis_angle = angle * axis
        
        return axis_angle

    def compute_velocity(self, Rs: list[np.ndarray], ts: list[np.ndarray], delta_t: float) -> tuple[np.ndarray, np.ndarray]:
        """ See KISS-ICP equation (1) """
        if len(Rs) < 2:
            return np.zeros(3), np.zeros(3)

        # a = Rs[-2].T @ Rs[-1]
        # b = (Rs[-2].T @ (ts[-1] - ts[-2])).reshape(-1,1)

        # c = np.hstack([a,b])
        # T_pred = np.vstack([c, [0,0,0,1]]) # calculated easily, but what is this used for ???

        v = (Rs[-2].T @ (ts[-1] - ts[-2])) / delta_t
        w = self.log_rotation(Rs[-2].T @ Rs[-1]) / delta_t

        return v, w


    def scan_callback(self, scan: LaserScan):
        """ Take a scan and estimate linear and angular velocity"""

        # Remove points that are inf
        all_ranges = np.array(scan.ranges)

        non_inf_indices = ~np.isinf(all_ranges)
        
        ranges = all_ranges[non_inf_indices] # convoluted way of removing rows with -inf or inf
        angles = scan.angle_min + np.where(non_inf_indices)[0] * scan.angle_increment # get the indices of the ranges that are kept

        # Update the target and current points
        self.x = self.compute_world_coords(ranges, angles)

        # Set y if this is our first run
        if self.y is None:
            self.y = self.x
            return

        R, t = self.ICP(self.x, self.y)
        
        self.Rs.append(R)
        self.ts.append(t)

        # TODO: Currently a problem since scan.scan_time, scan.time_increment are both 0 for this simulated LIDAR - gives junk
        v, w = calculate_velocities(self.Rs[-2],self.ts[-2], self.Rs[-1], self.ts[-1], 1.0)
        # v,w = self.compute_velocity(self.Rs, self.ts, 1.0)

        # self.get_logger().info(f'\nI heard:\n {scan.time_increment}, \nv:\n{v}, \nw:\n{w}, \nR:\n{R}, \nt:\n{t}')
        print(f"{v[0]:.2f}, \t{v[1]:.2f}")

        self.y = self.x


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CalculateOdometry()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()