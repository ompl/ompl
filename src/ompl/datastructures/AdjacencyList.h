/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#ifndef ADJACENCY_LIST_
#define ADJACENCY_LIST_

#include <vector>
#include <boost/thread/mutex.hpp>

namespace ompl
{
    // An undirected graph containing vertices and weighted edges
    // Edges are stored using an adjacency list.
    // Only one edge per vertex pair is allowed.
    class AdjacencyList
    {
        public:
            AdjacencyList();

            // Initialize the graph with n vertices and no edges
            AdjacencyList(int n);

            ~AdjacencyList();

            // Remove all vertices and edges
            void clear();

            // Add a new vertex to the graph.  The id of the vertex is returned
            // Note: Vertex removal is not supported to simplify the implementation,
            //       provide stability of vertex descriptors, and for performance reasons
            int addVertex();
            // Return the number of vertices in the graph
            int numVertices() const;
            // Return true if a vertex with the given id exists
            bool vertexExists(int v) const;

            // Return true if v1 and v2 are connected in the graph (in the same connected component)
            // NOTE: THIS FUNCTION MAY NOT BE CORRECT IF EDGES ARE EVER REMOVED FROM THE GRAPH
            bool inSameComponent(int v1, int v2) const;
            // Return the number of connected components in the graph
            // NOTE: THIS FUNCTION MAY NOT BE CORRECT IF EDGES ARE EVER REMOVED FROM THE GRAPH
            int numConnectedComponents() const;
            // Return the connected component set the vertex belongs to
            int getComponentID(int vtx) const;

            // Add an edge between v1 and v2 in the graph with the (optional) weight
            // If the edge already exists, false is returned
            bool addEdge(int v1, int v2, double weight = 1.0);
            // Remove the edge between v1 and v2.  False is returned if the edge does not exist
            // NOTE: USING THIS FUNCTION TRASHES CONNECTED COMPONENT CAPABILITIES
            bool removeEdge(int v1, int v2);
            // Return the number of edges in the graph
            int numEdges() const;
            // Returns the weight of the edge between vertices v1 and v2.  An exception is thrown when the edge does not exist
            double getEdgeWeight(int v1, int v2) const;
            // Update the edge weight between v1 and v2
            bool setEdgeWeight(int v1, int v2, double weight);
            // Returns true if an edge exists between vertices v1 and v2
            bool edgeExists(int v1, int v2) const;

            // Returns number of adjacent vertices for the given vertex
            int numNeighbors(int vtx) const;
            // Returns the adjacent vertices of the given vertex
            void getNeighbors(int vtx, std::vector<int>& nbrs) const;
            // Return the adjacent vertices of the given vertex and the weights associated with each edge
            void getNeighbors(int vtx, std::vector<std::pair<int, double> >& nbrs) const;

            // Dijkstra's shortest path search from v1 to v2.  If a path exists, the solution is stored in path and true is returned.
            bool dijkstra(int v1, int v2, std::vector<int>& path) const;

            // Dijkstra's shortest path search starting from v1.  The predecessors of each node are stored in predecessors.
            // If the predecessor of a node is itself, it is not reachable from vtx (or is equal to vtx).
            // The total distance from vtx to each node is stored in distance.  A value of double::max() indicates an unreachable node.
            void dijkstra(int vtx, std::vector<int>& predecessors, std::vector<double>& distance) const;

        protected:

            // Mutex, for thread safety
            mutable boost::mutex lock_;

            // Obscured to prevent unnecessary inclusion of BGL throughout the rest of the code.
            void* graphRaw_;

            // Obscured to prevent unnecessary inclusion of BGL throughout the rest of the code.
            void* disjointSetsRaw_;
    };
}

#endif
