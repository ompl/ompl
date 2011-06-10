
The following algorithms are implemented in this contribution (see the documentation for more information):


1. RRT* :
RRT* (optimal RRT) is an asymptotically-optimal incremental sampling-based motion planning algorithm. RRT* algorithm is guaranteed to converge to an optimal solution, while its running time is guaranteed to be a constant factor of the running time of the RRT".


2. Ball Tree RRT*:
Implementation of RRT* that incorporates Ball Trees to approximate connected regions of free space with volumes in configuration space instead of points. Every vertex added to the tree has an initial volume of an infinite radius associated with it. This radius is gradually reduced as collisions are found. All samples within any of the existing volumes are discarded. However, discarded samples are collision checked. If a collision is found, the nearest volume is trimmed at the collision point. Information from all collision checking procedures within iterations is also used to trim volumes accordingly.