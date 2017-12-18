#!/usr/bin/env python

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

# Author: Ryan Luna

try:
    # graph-tool and py-OMPL have some minor issues coexisting with each other.  Both modules
    # define conversions to C++ STL containers (i.e. std::vector), and the module that is imported
    # first will have its conversions used.  Order doesn't seem to matter on Linux,
    # but on Apple, graph_tool will not be imported properly if OMPL comes first.
    import graph_tool.all as gt
    graphtool = True
except ImportError:
    print('Failed to import graph-tool.  PlannerData will not be analyzed or plotted')
    graphtool = False

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

def useGraphTool(pd):
    # Extract the graphml representation of the planner data
    graphml = pd.printGraphML()
    f = open("graph.graphml", 'w')
    f.write(graphml)
    f.close()

    # Load the graphml data using graph-tool
    graph = gt.load_graph("graph.graphml", fmt="xml")
    edgeweights = graph.edge_properties["weight"]

    # Write some interesting statistics
    avgdeg, stddevdeg = gt.vertex_average(graph, "total")
    avgwt, stddevwt = gt.edge_average(graph, edgeweights)

    print("---- PLANNER DATA STATISTICS ----")
    print(str(graph.num_vertices()) + " vertices and " + str(graph.num_edges()) + " edges")
    print("Average vertex degree (in+out) = " + str(avgdeg) + "  St. Dev = " + str(stddevdeg))
    print("Average edge weight = " + str(avgwt)  + "  St. Dev = " + str(stddevwt))

    _, hist = gt.label_components(graph)
    print("Strongly connected components: " + str(len(hist)))

    # Make the graph undirected (for weak components, and a simpler drawing)
    graph.set_directed(False)
    _, hist = gt.label_components(graph)
    print("Weakly connected components: " + str(len(hist)))

    # Plotting the graph
    gt.remove_parallel_edges(graph) # Removing any superfluous edges

    edgeweights = graph.edge_properties["weight"]
    colorprops = graph.new_vertex_property("string")
    vertexsize = graph.new_vertex_property("double")

    start = -1
    goal = -1

    for v in range(graph.num_vertices()):

        # Color and size vertices by type: start, goal, other
        if pd.isStartVertex(v):
            start = v
            colorprops[graph.vertex(v)] = "cyan"
            vertexsize[graph.vertex(v)] = 10
        elif pd.isGoalVertex(v):
            goal = v
            colorprops[graph.vertex(v)] = "green"
            vertexsize[graph.vertex(v)] = 10
        else:
            colorprops[graph.vertex(v)] = "yellow"
            vertexsize[graph.vertex(v)] = 5

    # default edge color is black with size 0.5:
    edgecolor = graph.new_edge_property("string")
    edgesize = graph.new_edge_property("double")
    for e in graph.edges():
        edgecolor[e] = "black"
        edgesize[e] = 0.5

    # using A* to find shortest path in planner data
    if start != -1 and goal != -1:
        _, pred = gt.astar_search(graph, graph.vertex(start), edgeweights)

        # Color edges along shortest path red with size 3.0
        v = graph.vertex(goal)
        while v != graph.vertex(start):
            p = graph.vertex(pred[v])
            for e in p.out_edges():
                if e.target() == v:
                    edgecolor[e] = "red"
                    edgesize[e] = 2.0
            v = p

    # Writing graph to file:
    # pos indicates the desired vertex positions, and pin=True says that we
    # really REALLY want the vertices at those positions
    gt.graph_draw(graph, vertex_size=vertexsize, vertex_fill_color=colorprops,
                  edge_pen_width=edgesize, edge_color=edgecolor,
                  output="graph.png")
    print('\nGraph written to graph.png')

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
    start = ob.State(space)
    start().setX(-9)
    start().setY(-9)
    start().setZ(-9)
    start().rotation().setIdentity()

    # create a goal state
    goal = ob.State(space)
    goal().setX(-9)
    goal().setY(9)
    goal().setZ(-9)
    goal().rotation().setIdentity()

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # Lets use PRM.  It will have interesting PlannerData
    planner = og.PRM(ss.getSpaceInformation())
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

        if graphtool:
            useGraphTool(pd)

if __name__ == "__main__":
    plan()
