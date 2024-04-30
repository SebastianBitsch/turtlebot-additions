import numpy as np

def project_point_to_linesegment(start: np.ndarray, end: np.ndarray, p: np.ndarray) -> np.ndarray:
    """
    Vectorized operation for projecting a point onto on array of line segments.
    Follows the procedure outlined in: https://stackoverflow.com/a/1501725/19877091

    Parameters:
        start (np.ndarray) (N, 2): Starting point of the line segments.
        end (np.ndarray) (N, 2): Ending point of the line segments.
        p (np.ndarray) (2, ): Point to be projected onto the line segments.

    Returns:
        np.ndarray: The projections of point p onto the line segments defined by start and end.
        np.ndarray: The distance [0, 1] along the line segment where the projection lies
    """
    l2 = np.linalg.norm(start - end, axis=1)**2 # TODO: Not very fast

    dp = np.sum((p - start) * (end - start), axis=-1) / l2
    
    ones  = np.ones(len(start))
    zeros = np.zeros(len(start))

    t = np.max([zeros, np.min([ones, dp], axis=0)], axis=0)
    proj = start + t[:,None] * (end - start)

    return proj, t


def RDT(initial_point: np.ndarray = None, bounds: np.ndarray = np.ones(2), n: int = 1000):
    """
    Generate a Rapidly-exploring Dense Tree (RDT) following Figure 5.16 in 'Planning algorithms'.

    Parameters:
        initial_point (np.ndarray, optional): Initial point for the tree. Defaults to None and will generate from the center.
        bounds (np.ndarray, optional): Bounds for the space in which the tree is generated. Defaults to 1 x 1.
        n (int, optional): Number of line segments to generate in the tree. Defaults to 1000.

    Returns:
        np.ndarray (N x 2 x 2): Array containing line segments representing the RDT. Please note the function either returns N or N-1 line segments.
    """
    k = 1
    if initial_point is None:
        initial_point = bounds / 2.0

    # Create an initial first line segment
    lines = np.zeros((n, 2, 2)) # n X first point (x,y) X second point (x,y)
    lines[0] = np.vstack([
        initial_point, 
        np.random.rand(2) * bounds
    ])

    while k < n - 1:
        # Generate a new point
        point = np.random.rand(2) * bounds
        
        # Project the new point onto all the current line segments
        projections, t = project_point_to_linesegment(lines[:k, 0], lines[:k, 1], point)

        # Get the closest projection 
        dists = np.linalg.norm(point - projections, axis=1)        
        closest = np.argmin(dists)

        # Add line segment between new point and projection
        lines[k] = np.vstack([point, projections[closest]])
        
        # Split line into two line segments if the projection doesnt lie at one of the end points
        if t[closest] != 0.0 and t[closest] != 1.0:
            lines[k+1] = np.vstack([
                lines[closest, 1],
                projections[closest]
            ])
            lines[closest] = np.vstack([
                lines[closest, 0],
                projections[closest]
            ])
            k += 1
        k += 1

    return lines[:k] # doesn't guarantee we return exactly n lines, might be n - 1. Dont know if this is undesired


if __name__ == "__main__":
    np.random.seed(0)
    	
    # Setup plot
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5,5), dpi = 300)
    ax.set_xlim([0,1])
    ax.set_ylim([0,1])
    ax.set_axis_off()

    lines = RDT(n = 100)

    # Draw plot
    for start, end in lines:
        ax.plot([start[0], end[0]], [start[1], end[1]], color='black')

    plt.show()