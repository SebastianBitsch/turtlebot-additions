import rclpy
from rclpy.node import Node

import numpy as np
from sklearn.neighbors import NearestNeighbors

from sensor_msgs.msg import LaserScan


class CalculateOdometry(Node):

    def __init__(self):
        super().__init__('calculate_odometry')

        # time series of rotation matrices and transformation vectors
        self.Rs = [np.eye(3)]
        self.ts = [np.zeros(3)]
        self.x = None
        self.y = None

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning, not sure needed

    def transform(self, x: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """ rotate and move a vector, not sure transform is the right word """
        assert R.shape == (3,3) and t.shape == (3,) and x.shape[1] == 3, "Wrong shapes"
        return (R @ x.T + t[:, None]).T

    def compute_world_coords(self, scan: LaserScan) -> np.ndarray:
        """ Assumes 2D scanner, i.e. (probably) wont work in 3D """
        angles = np.arange(scan.angle_min, scan.angle_max, step = scan.angle_increment)
        x = np.array(scan.ranges) * np.cos(angles)
        y = np.array(scan.ranges) * np.sin(angles)
        z = np.zeros(len(scan.ranges))

        # "rotate" by -90 degrees to get 0 degrees to be forwards
        # replace x=-y with x=y to flip around x axis, i.e. scan direction clockwise
        # can be seen in the header of the scan
        # this can just be done directly in the first two lines instead of here, just like this for now
        temp = x
        x = -y
        y = temp

        coords = np.vstack([x, y, z]).T # [360, 3] for instance
        return coords
    
    def weighted_average(self, values: np.ndarray, weights: np.ndarray = None) -> np.ndarray:
        """ Get the centre of mass for a set of values, """
        assert weights is None or values.shape[0] == weights.shape[0]
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
    

    def ICP(self, x: np.ndarray, y: np.ndarray, max_iters: int = 10) -> tuple[np.ndarray, np.ndarray]:
        """ Does vanilla point-to-point ICP. Not KISS-ICP, too hard tbh """
        R = np.eye(3)
        t = np.zeros(3)
        distance_threshold = 100.0 # should ideally not be a parameter, makes it hard to tune
        tolerance = 0.2  # idk a good value

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(y)

        # Iterative loop
        for _ in range(max_iters):
            # Find the nearest neighbors for each point in the moving point cloud x
            distances, indices = nbrs.kneighbors(self.transform(x,R,t))

            x_closest, y_closest = self.compute_closest_pairs(x, y, distances, indices, distance_threshold)

            R, t = self.compute_transformation_params(x_closest, y_closest, weights=None)
            
            x_hat = self.transform(x,R,t)

            # Convergence check
            if np.allclose(x_hat, y, rtol = tolerance): # idk about the tolerance parameter, should visualize to find a good value
                break

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

    def compute_velocity(self, Rs: np.ndarray, ts: np.ndarray, delta_t: float) -> np.ndarray:
        """ See KISS-ICP equation (1) """
        a = Rs[-2].T @ Rs[-1]
        b = (Rs[-2].T @ (ts[-1] - ts[-2])).reshape(-1,1)

        c = np.hstack([a,b])
        T_pred = np.vstack([c, [0,0,0,1]]) # calculated easily, but what is this used for ???

        v = (Rs[-2].T @ (ts[-1] - ts[-2])) / delta_t
        w = self.log_rotation(Rs[-2].T @ Rs[-1]) / delta_t

        return v, w


    def scan_callback(self, scan: LaserScan):
        """ Take a scan and estimate linear and angular velocity"""

        # Update the target and current points
        self.x = self.compute_world_coords(scan)

        # remove points that are inf
        self.x = self.x[np.all(self.x < np.inf,axis=1)]
        print(self.x)
        print(self.x.shape)
        print(self.y.shape)
        if self.y is None:
            self.y = self.x.copy()

        R, t = self.ICP(self.x, self.y)
        
        self.Rs.append(R)
        self.ts.append(t)

        v,w = self.compute_pose_estimate(self.Rs, self.ts, scan.scan_time)

        self.get_logger().info(f'I heard: {scan.header}, v:{v}, w:{w}, R:{R}, t:{t}')

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