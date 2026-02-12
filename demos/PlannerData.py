#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Ryan Luna, Weihang Guo

try:
    import viser
    import numpy as np
    viser_available = True
except ImportError:
    print('Failed to import viser.  PlannerData will not be visualized')
    viser_available = False

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og


# Create a narrow passage between y=[-3,3].  Only a 6x6x6 cube will be valid, centered at origin
def isStateValid(state):
    if state.getY() >= -3 and state.getY() <= 3:
        return state.getX() >= -3 and state.getX() <= 3 and \
            state.getZ() >= -3 and state.getZ() <= 3
    return True

def useViser(pd):
    """
    Visualize planner data graph in 3D using viser.
    Nodes are displayed at their actual 3D positions, edges as line segments.
    """
    num_vertices = pd.numVertices()
    num_edges = pd.numEdges()
    
    if num_vertices == 0:
        print("No vertices in planner data to visualize")
        return
    
    # Extract 3D positions and build graph structure
    positions = []
    vertex_colors = []
    vertex_sizes = []
    start_indices = []
    goal_indices = []
    
    # Build adjacency list and edge weights
    adjacency = [[] for _ in range(num_vertices)]
    edge_weights = {}
    
    for v in range(num_vertices):
        vertex = pd.getVertex(v)
        state = vertex.getState()
        
        # Extract 3D position (assuming SE3StateSpace)
        x = state.getX()
        y = state.getY()
        z = state.getZ()
        positions.append([x, y, z])
        
        # Determine vertex color and size
        if pd.isStartVertex(v):
            start_indices.append(v)
            vertex_colors.append([0, 1, 1])  # cyan
            vertex_sizes.append(0.3)
        elif pd.isGoalVertex(v):
            goal_indices.append(v)
            vertex_colors.append([0, 1, 0])  # green
            vertex_sizes.append(0.3)
        else:
            vertex_colors.append([1, 1, 0])  # yellow
            vertex_sizes.append(0.15)
        
        # Get outgoing edges
        outgoing = pd.getEdges(v)
        for target in outgoing:
            if pd.edgeExists(v, target):
                # Compute edge weight as distance between states
                state1 = pd.getVertex(v).getState()
                state2 = pd.getVertex(target).getState()
                si = pd.getSpaceInformation()
                weight = si.distance(state1, state2)
                adjacency[v].append((target, weight))
                edge_weights[(v, target)] = weight
    
    positions = np.array(positions, dtype=np.float32)
    vertex_colors = np.array(vertex_colors, dtype=np.float32)
    
    # Compute statistics
    degrees = [len(adjacency[v]) + len(pd.getIncomingEdges(v)) for v in range(num_vertices)]
    avg_degree = np.mean(degrees) if degrees else 0
    stddev_degree = np.std(degrees) if degrees else 0
    
    all_weights = list(edge_weights.values())
    avg_weight = np.mean(all_weights) if all_weights else 0
    stddev_weight = np.std(all_weights) if all_weights else 0
    
    # Compute connected components (using DFS)
    def dfs_component(v, visited, component):
        visited[v] = True
        component.append(v)
        for neighbor, _ in adjacency[v]:
            if not visited[neighbor]:
                dfs_component(neighbor, visited, component)
        # Also check incoming edges (for undirected graph)
        for neighbor in pd.getIncomingEdges(v):
            if not visited[neighbor]:
                dfs_component(neighbor, visited, component)
    
    visited = [False] * num_vertices
    components = []
    for v in range(num_vertices):
        if not visited[v]:
            component = []
            dfs_component(v, visited, component)
            components.append(component)
    
    print("---- PLANNER DATA STATISTICS ----")
    print(f"{num_vertices} vertices and {num_edges} edges")
    print(f"Average vertex degree (in+out) = {avg_degree:.3f}  St. Dev = {stddev_degree:.3f}")
    print(f"Average edge weight = {avg_weight:.3f}  St. Dev = {stddev_weight:.3f}")
    print(f"Connected components: {len(components)}")
    
    # Find shortest path using Dijkstra's algorithm
    path_edges = set()
    if start_indices and goal_indices:
        # Use first start and first goal
        start = start_indices[0]
        goal = goal_indices[0]
        
        # Dijkstra's algorithm
        import heapq
        dist = {v: float('inf') for v in range(num_vertices)}
        prev = {v: None for v in range(num_vertices)}
        dist[start] = 0
        pq = [(0, start)]
        
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == goal:
                break
            
            # Check outgoing edges
            for v, weight in adjacency[u]:
                alt = dist[u] + weight
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(pq, (alt, v))
            
            # Check incoming edges (for undirected graph)
            for v in pd.getIncomingEdges(u):
                # Get weight for incoming edge by computing distance
                if pd.edgeExists(v, u):
                    state1 = pd.getVertex(v).getState()
                    state2 = pd.getVertex(u).getState()
                    si = pd.getSpaceInformation()
                    weight = si.distance(state1, state2)
                    alt = dist[u] + weight
                    if alt < dist[v]:
                        dist[v] = alt
                        prev[v] = u
                        heapq.heappush(pq, (alt, v))
        
        # Reconstruct path
        if prev[goal] is not None or goal == start:
            path = []
            current = goal
            while current is not None:
                path.append(current)
                current = prev[current]
            path.reverse()
            
            # Mark path edges
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                path_edges.add((u, v))
                path_edges.add((v, u))  # Add reverse for undirected
    
    # Create viser server
    server = viser.ViserServer(port=8080, label="OMPL Planner Data Visualization")
    
    # Add vertices as point cloud
    server.scene.add_point_cloud(
        "/graph/vertices",
        points=positions,
        colors=vertex_colors,
        point_size=0.2
    )
    
    # Add edges as line segments
    # Separate path edges from regular edges for different styling
    regular_edge_lines = []
    path_edge_lines = []
    
    # Track which edges we've added to avoid duplicates
    added_edges = set()
    
    for v in range(num_vertices):
        for target, _ in adjacency[v]:
            # Only add each edge once (v < target to avoid duplicates)
            if (v, target) not in added_edges:
                added_edges.add((v, target))
                p1 = positions[v]
                p2 = positions[target]
                
                # Check if this edge is part of the path
                if (v, target) in path_edges or (target, v) in path_edges:
                    path_edge_lines.append([p1, p2])
                else:
                    regular_edge_lines.append([p1, p2])
    
    # Add regular edges (black, thinner)
    if regular_edge_lines:
        regular_edge_lines = np.array(regular_edge_lines, dtype=np.float32)
        # Reshape to (N, 2, 3) format expected by viser
        regular_edge_lines = regular_edge_lines.reshape(-1, 2, 3)
        # Use a single color tuple (3,) which applies to all segments
        server.scene.add_line_segments(
            "/graph/edges/regular",
            points=regular_edge_lines,
            colors=(0, 0, 0),  # black
            line_width=0.5
        )
    
    # Add path edges (red, thicker)
    if path_edge_lines:
        path_edge_lines = np.array(path_edge_lines, dtype=np.float32)
        # Reshape to (N, 2, 3) format expected by viser
        path_edge_lines = path_edge_lines.reshape(-1, 2, 3)
        # Use a single color tuple (3,) which applies to all segments
        server.scene.add_line_segments(
            "/graph/edges/path",
            points=path_edge_lines,
            colors=(1, 0, 0),  # red
            line_width=3.0
        )
    
    print('\n3D graph visualization available at http://localhost:8080')
    print('Press Ctrl+C to stop the server')
    
    # Keep server running
    try:
        server.sleep_forever()
    except KeyboardInterrupt:
        server.stop()
        print('\nServer stopped')

def plan():
    # construct the state space we are planning in
    space = ob.SE3StateSpace()

    # set the bounds for R^3 portion of SE(3)
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-10)
    bounds.setHigh(10)
    space.setBounds(bounds)

    # define a simple setup class
    ss = og.SimpleSetup(space)

    # create a start state
    start = space.allocState()
    start.setX(-9)
    start.setY(-9)
    start.setZ(-9)
    start.rotation().setIdentity()

    # create a goal state
    goal = space.allocState()
    goal.setX(-9)
    goal.setY(9)
    goal.setZ(-9)
    goal.rotation().setIdentity()

    ss.setStateValidityChecker(isStateValid)

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # planner = og.RRTConnect(ss.getSpaceInformation())
    planner = og.PRM(ss.getSpaceInformation())
    # planner = og.RRT(ss.getSpaceInformation())
    
    ss.setPlanner(planner)
    ss.setup()

    # attempt to solve the problem
    solved = ss.solve(20.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath())

        # Extracting planner data from most recent solve attempt
        pd = ob.PlannerData(ss.getSpaceInformation())
        ss.getPlannerData(pd)

        # Computing weights of all edges based on state space distance
        pd.computeEdgeWeights()

        if viser_available:
            useViser(pd)

if __name__ == "__main__":
    plan()
