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

#define BOOST_TEST_MODULE "AdjacencyList"
#include <boost/test/unit_test.hpp>
#include <iostream>

#include "ompl/datastructures/AdjacencyList.h"

#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

// Simple graph construction.  No removal of elements
BOOST_AUTO_TEST_CASE(SimpleConstruction)
{
    ompl::AdjacencyList graph;
    BOOST_CHECK_EQUAL(graph.numVertices(), 0);
    BOOST_CHECK_EQUAL(graph.numEdges(), 0);

    // Creating 1000 vertices
    int numVertices = 1000;
    for (int i = 0; i < numVertices; ++i)
    {
        BOOST_CHECK(!graph.vertexExists(i));
        BOOST_CHECK_EQUAL(graph.addVertex(), i);
        BOOST_CHECK(graph.vertexExists(i));
        BOOST_CHECK_EQUAL(graph.numNeighbors(i), 0);
    }

    // We should have #states vertices and 0 edges
    BOOST_CHECK_EQUAL(graph.numVertices(), numVertices);
    BOOST_CHECK_EQUAL(graph.numEdges(), 0 );
    BOOST_CHECK_EQUAL(graph.numConnectedComponents(), numVertices);

    // Adding edges - making a ring topology
    for (int i = 0; i < numVertices; ++i)
    {
        int source = i;
        int target = (i+1) % numVertices;

        BOOST_CHECK(!graph.edgeExists(source, target));
        BOOST_CHECK(graph.addEdge(source, target, std::max(source, target)));
        BOOST_CHECK(graph.edgeExists(source, target));
        BOOST_CHECK(graph.edgeExists(target, source)); // undirected edges

        BOOST_CHECK_EQUAL(graph.numNeighbors(source), (source == 0 ? 1 : 2));

        double weight = graph.getEdgeWeight(source, target);
        BOOST_OMPL_EXPECT_NEAR(weight, std::max(source, target), 1e-6);

        weight = graph.getEdgeWeight(target, source);
        BOOST_OMPL_EXPECT_NEAR(weight, std::max(source, target), 1e-6);

        if (i != numVertices-1)
        {
            BOOST_CHECK_EQUAL(graph.numConnectedComponents(), numVertices - i - 1);
            for(int j = 0; j < target; ++j)
                BOOST_CHECK(graph.inSameComponent(i, j));
            for(int j = target+1; j < numVertices; ++j)
                BOOST_CHECK(!graph.inSameComponent(i, j));
        }
        else
            BOOST_CHECK_EQUAL(graph.numConnectedComponents(), 1);
    }

    // We should have #states vertices and 0 edges
    BOOST_CHECK_EQUAL(graph.numVertices(), numVertices);
    BOOST_CHECK_EQUAL(graph.numEdges(), numVertices);

    // Make sure the neighbors are who we think they are
    for (int i = 0; i < numVertices; ++i)
    {
        BOOST_CHECK_EQUAL(graph.numNeighbors(i), 2);

        std::vector<int> nbrs;
        graph.getNeighbors(i, nbrs);

        // make sure all neighbors are distinct
        for(size_t j = 0; j < nbrs.size(); ++j)
            for(size_t k = j+1; k < nbrs.size(); ++k)
                BOOST_REQUIRE(nbrs[j] != nbrs[k]);

        // Make sure the neighbors are correct
        int nbr1 = (i+1) % numVertices;
        int nbr2 = (i == 0 ? numVertices - 1 : i-1);
        BOOST_CHECK(nbrs[0] == nbr1 || nbrs[0] == nbr2);
        BOOST_CHECK(nbrs[1] == nbr1 || nbrs[1] == nbr2);
    }

    // Check edge weights
    for (int i = 0; i < numVertices; ++i)
    {
        BOOST_CHECK_EQUAL(graph.numNeighbors(i), 2);

        std::vector<std::pair<int, double> > nbrWeights;
        graph.getNeighbors(i, nbrWeights);
        BOOST_CHECK(nbrWeights.size() == 2);

        // make sure all neighbors are distinct
        for(size_t j = 0; j < nbrWeights.size(); ++j)
            for(size_t k = j+1; k < nbrWeights.size(); ++k)
                BOOST_REQUIRE(nbrWeights[j].first != nbrWeights[k].first);

        // Make sure the neighbors are correct
        int nbr1 = (i+1) % numVertices;
        int nbr2 = (i == 0 ? numVertices - 1 : i-1);
        BOOST_CHECK(nbrWeights[0].first == nbr1 || nbrWeights[0].first == nbr2);
        BOOST_CHECK(nbrWeights[1].first == nbr1 || nbrWeights[1].first == nbr2);

        // make sure weights are correct
        for(size_t j = 0; j < nbrWeights.size(); ++j)
        {
            double weight = graph.getEdgeWeight(i, nbrWeights[j].first);
            double expected = std::max(nbrWeights[j].first, i);
            BOOST_OMPL_EXPECT_NEAR(weight, expected, 1e-6);
        }
    }

    graph.clear();
    BOOST_CHECK_EQUAL(graph.numVertices(), 0);
    BOOST_CHECK_EQUAL(graph.numEdges(), 0);
    BOOST_CHECK_EQUAL(graph.numConnectedComponents(), 0);
}

BOOST_AUTO_TEST_CASE(AddModifyRemove)
{
    ompl::AdjacencyList graph;

    // Creating 1000 vertices
    int numVertices = 1000;
    for (int i = 0; i < numVertices; ++i)
        BOOST_CHECK_EQUAL(graph.addVertex(), i);

    // We should have #states vertices and 0 edges
    BOOST_CHECK_EQUAL(graph.numVertices(), numVertices);
    BOOST_CHECK_EQUAL(graph.numEdges(), 0 );


    // Adding edges - making a ring topolocy
    for (int i = 0; i < numVertices; ++i)
    {
        int source = i;
        int target = (i+1) % numVertices;
        BOOST_CHECK(graph.addEdge(source, target, std::max(source, target)));  // adding edge with weight i+1
    }

    // Add edges to 0 from all nodes
    for (int i = 0; i < numVertices; ++i)
    {
        int source = i;
        int target = 0;

        // Don't try to add an edge that exists
        bool exists = graph.edgeExists(source, target);
        if (exists)
        {
            // Make sure we expect this edge
            BOOST_CHECK(target == (i > 0 ? i - 1 : numVertices -1) ||
                        target == (i < numVertices - 1 ? i + 1 : 0));
        }
        else
        {
            if (source != target)
                BOOST_CHECK(graph.addEdge(source, target, i));  // adding edge with weight i
            else BOOST_CHECK(!graph.addEdge(source, target, i));  // NOT adding edge with weight i (self transitions not allowed)
            BOOST_CHECK(!graph.addEdge(source, target, i));  // NOT adding edge again with weight i
        }
    }

    // Check size of graph
    BOOST_CHECK_EQUAL(graph.numVertices(), numVertices);
    BOOST_CHECK_EQUAL(graph.numEdges(), numVertices + numVertices - 3);

    // Make sure the neighbors are who we think they are
    for (int i = 0; i < numVertices; ++i)
    {
        std::vector<int> nbrs;
        graph.getNeighbors(i, nbrs);
        BOOST_CHECK_EQUAL(nbrs.size(), graph.numNeighbors(i));

        // make sure all neighbors are distinct
        for(size_t j = 0; j < nbrs.size(); ++j)
            for(size_t k = j+1; k < nbrs.size(); ++k)
                BOOST_REQUIRE(nbrs[j] != nbrs[k]);

        // Make sure the neighbors are correct
        if (i == 0)
        {
            BOOST_CHECK_EQUAL(graph.numNeighbors(i), numVertices-1);
            for(size_t j = 0; j < nbrs.size(); ++j)
                BOOST_CHECK(nbrs[j] != 0);
        }
        else if (i == 1 || i == numVertices - 1)
        {
            BOOST_CHECK_EQUAL(graph.numNeighbors(i), 2);

            int nbr1 = (i == 1 ? 0 : i-1);
            int nbr2 = (i == 1 ? i+1 : 0);
            for(size_t j = 0; j < nbrs.size(); ++j)
                BOOST_CHECK(nbrs[j] == nbr1 || nbrs[j] == nbr2);
        }
        else
        {
            BOOST_CHECK_EQUAL(graph.numNeighbors(i), 3);

            int nbr1 = i+1;
            int nbr2 = i-1;
            int nbr3 = 0;
            for(size_t j = 0; j < nbrs.size(); ++j)
                BOOST_CHECK(nbrs[j] == nbr1 || nbrs[j] == nbr2 || nbrs[j] == nbr3);
        }
    }

    BOOST_CHECK_EQUAL(graph.numEdges(), numVertices + numVertices - 3);

    // Change edge weights
    for(int i = 0; i < numVertices; ++i)
    {
        for(int j = i+1; j < numVertices; ++j)
        {
            if (graph.edgeExists(i, j))
            {
                double expWeight = std::max(i, j);
                double weight = graph.getEdgeWeight(i, j);
                BOOST_OMPL_EXPECT_NEAR(weight, expWeight, 1e-6);

                double newWeight = std::min(i, j);
                graph.setEdgeWeight(i, j, newWeight);
                weight = graph.getEdgeWeight(i, j);
                BOOST_OMPL_EXPECT_NEAR(weight, newWeight, 1e-6);

            }
        }
    }

    int numEdges = graph.numEdges();

    // remove edges
    for(int i = 1; i < numVertices-1; ++i)
    {
        if (i == 1 || i == numVertices -1)
            BOOST_REQUIRE_EQUAL(graph.numNeighbors(i), 2);
        else
            BOOST_REQUIRE_EQUAL(graph.numNeighbors(i), 3);

        BOOST_REQUIRE(graph.edgeExists(0, i));
        BOOST_REQUIRE(graph.edgeExists(i, 0));

        BOOST_CHECK(graph.removeEdge(0, i));
        BOOST_REQUIRE(!graph.edgeExists(0, i));
        BOOST_REQUIRE(!graph.edgeExists(i, 0));
        BOOST_CHECK_EQUAL(graph.numEdges(), --numEdges);

        if (i == 1 || i == numVertices -1)
            BOOST_REQUIRE_EQUAL(graph.numNeighbors(i), 1);
        else
            BOOST_REQUIRE_EQUAL(graph.numNeighbors(i), 2);

        std::vector<int> nbrs;
        std::vector<std::pair<int, double> > nbrWeights;
        graph.getNeighbors(i, nbrs);
        graph.getNeighbors(i, nbrWeights);
        BOOST_CHECK_EQUAL(nbrs.size(), graph.numNeighbors(i));
        BOOST_CHECK_EQUAL(nbrWeights.size(), graph.numNeighbors(i));

        // make sure all neighbors are distinct
        for(size_t j = 0; j < nbrs.size(); ++j)
            for(size_t k = j+1; k < nbrs.size(); ++k)
                BOOST_REQUIRE(nbrs[j] != nbrs[k]);

        // make sure all neighbors are distinct
        for(size_t j = 0; j < nbrWeights.size(); ++j)
            for(size_t k = j+1; k < nbrWeights.size(); ++k)
                BOOST_REQUIRE(nbrWeights[j].first != nbrWeights[k].first);

        int nbr1 = i+1;
        int nbr2 = i-1;
        for(size_t j = 0; j < nbrs.size(); ++j)
            BOOST_CHECK(nbrs[j] == nbr1 || nbrs[j] == nbr2);
        for(size_t j = 0; j < nbrWeights.size(); ++j)
        {
            BOOST_CHECK(nbrWeights[j].first == nbr1 || nbrWeights[j].first == nbr2);
            BOOST_OMPL_EXPECT_NEAR(nbrWeights[j].second, std::min(i, nbrWeights[j].first), 1e-6);
        }
    }
}

BOOST_AUTO_TEST_CASE(Dijkstra)
{
    ompl::AdjacencyList graph;

    // Creating 1000 vertices
    int numVertices = 1000;
    for (int i = 0; i < numVertices; ++i)
        BOOST_CHECK_EQUAL(graph.addVertex(), i);

    // We should have #states vertices and 0 edges
    BOOST_CHECK_EQUAL(graph.numVertices(), numVertices);
    BOOST_CHECK_EQUAL(graph.numEdges(), 0 );

    // Adding edges - making a ring topolocy
    for (int i = 0; i < numVertices; ++i)
    {
        int source = i;
        int target = (i+1) % numVertices;
        BOOST_CHECK(graph.addEdge(source, target, 1));  // adding edge with weight 1
    }

    BOOST_CHECK_EQUAL(graph.numVertices(), numVertices);
    BOOST_CHECK_EQUAL(graph.numEdges(), numVertices);

    // Short hop around the ring
    std::vector<int> path;
    BOOST_CHECK(graph.dijkstra(0, numVertices-1, path));
    BOOST_REQUIRE(path.size() == 2);
    BOOST_CHECK(path[0] == 0);
    BOOST_CHECK(path[1] == numVertices - 1);

    // Remove the hop in the ring.  Path should go through all vertices in order
    BOOST_CHECK(graph.removeEdge(0, numVertices-1));
    BOOST_REQUIRE_EQUAL(graph.numEdges(), numVertices-1);
    BOOST_CHECK(graph.dijkstra(0, numVertices-1, path));
    BOOST_REQUIRE(path.size() == (unsigned int)numVertices);
    for(int i = 0; i < (int)path.size(); ++i)
        BOOST_CHECK_EQUAL(path[i], i);

    // Add the hop in the ring, but make it crazy high edge weight.  Path should go through all vertices in order
    BOOST_REQUIRE(graph.addEdge(0, numVertices-1, numVertices));
    BOOST_REQUIRE_EQUAL(graph.numEdges(), numVertices);
    BOOST_CHECK(graph.dijkstra(0, numVertices-1, path));
    BOOST_REQUIRE(path.size() == (unsigned int)numVertices);
    for(int i = 0; i < (int)path.size(); ++i)
        BOOST_CHECK_EQUAL(path[i], i);

    if (numVertices > 3)
    {
        BOOST_CHECK(graph.setEdgeWeight(0, numVertices - 1, 1.0));

        // Remove all neighbors of 3
        std::vector<int> nbrs;
        graph.getNeighbors(3, nbrs);
        for(size_t i = 0; i < nbrs.size(); ++i)
        {
            BOOST_CHECK(graph.edgeExists(3, nbrs[i]));
            BOOST_CHECK(graph.removeEdge(3, nbrs[i]));
            BOOST_CHECK(!graph.edgeExists(3, nbrs[i]));
            BOOST_CHECK(!graph.edgeExists(nbrs[i], 3));
        }
        BOOST_CHECK_EQUAL(graph.numNeighbors(3), 0);

        // There should be no path to vertex 3
        BOOST_CHECK(!graph.dijkstra(0, 3, path));
    }
}
