/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#ifndef OMPL_BASE_PLANNER_DATA_
#define OMPL_BASE_PLANNER_DATA_

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {
        /// \brief Base class for a vertex in the PlannerData structure.  All 
        /// derived classes must implement the clone and equivalence operators.
        /// It is assumed that each vertex in the PlannerData structure is 
        /// unique (i.e. no duplicates allowed).
        class PlannerDataVertex
        {
        public:
            PlannerDataVertex (const base::State* st, int tag = 0) : state_(st), tag_(tag) {}
            PlannerDataVertex (const PlannerDataVertex& rhs) : state_(rhs.state_), tag_(rhs.tag_) {}
            virtual ~PlannerDataVertex (void) {}

            virtual int  getTag (void) const { return tag_; }
            virtual void setTag (int tag) { tag_ = tag; }
            virtual const base::State* getState(void) const { return state_; }

            virtual PlannerDataVertex* clone (void) const
            {
                return new PlannerDataVertex(*this);
            }

            virtual bool operator == (const PlannerDataVertex &rhs) const
            {
                // States should be unique
                return state_ == rhs.state_;
            }

        protected:
            const base::State* state_;
            int tag_;
        };

        /// \brief Base class for a PlannerData edge.
        class PlannerDataEdge
        {
        public:
            PlannerDataEdge (void) {}
            virtual ~PlannerDataEdge (void) {}
            virtual PlannerDataEdge* clone () const { return new PlannerDataEdge(); }
        };
        
        class Graph;
        ClassForward(PlannerData);

        /// Object containing planner generated vertex and edge data.
        class PlannerData : boost::noncopyable
        {
        public:
            // Constructor.  The name of the planner creating this structure is supplied.
            PlannerData(void);
            virtual ~PlannerData(void);

            // Clears the entire data structure
            void clear (void);
            // Check whether an edge between v1 and v2 exists
            bool edgeExists (unsigned int v1, unsigned int v2) const;
            // Check whether a vertex exists with the given data
            bool vertexExists (const PlannerDataVertex &v) const;

            // Retrieve a reference to the vertex object with the given index
            const PlannerDataVertex& getVertex (unsigned int index) const;
            // Retrieve a reference to the edge object connecting vertices v1 and v2
            const PlannerDataEdge& getEdge (unsigned int v1, unsigned int v2) const;
            // Retrieve a reference to the vertex object with the given index
            PlannerDataVertex& getVertex (unsigned int index);
            // Retrieve a reference to the edge object connecting vertices v1 and v2
            PlannerDataEdge& getEdge (unsigned int v1, unsigned int v2);

            // Returns a list of the vertices directly connected to vertex v.  The number of edges is returned.
            unsigned int getEdges (unsigned int v, std::vector<unsigned int>& edgeList) const;
            // Returns a map of out-going edges from vertex v.  Key = vertex ID,
            // value = edge structure.  The number of edges is returned.
            unsigned int getEdges (unsigned int v, std::map<unsigned int, const PlannerDataEdge*> &edgeMap) const;
            // Returns the weight of the edge between the given vertex ids.  -1.0 is returned for an invalid edge.
            double getEdgeWeight(unsigned int v1, unsigned int v2) const;
            // Sets the weight of the edge between the given vertex ids.  If an edge between v1 and v2 does not exist, false is returned.
            bool setEdgeWeight(unsigned int v1, unsigned int v2, double weight);
            // Retrieve the number of edges in this structure
            unsigned int numEdges (void) const;
            // Retrieve the number of vertices in this structure
            unsigned int numVertices (void) const;
            // Writes a Graphviz dot file of this structure to the given output stream
            void printGraphviz (std::ostream& out = std::cout) const;
            // Return the index for the vertex associated with the given data.  
            // unsigned int max is returned if this vertex does not exist in the
            // data.  O(n) complexity in the number of vertices.
            unsigned int vertexIndex (const PlannerDataVertex &v) const;

            // Adds the given vertex to the graph data.  A unique ID is returned.
            unsigned int addVertex (const PlannerDataVertex &st);
            // Removes the vertex associated with the given datum
            bool removeVertex (const PlannerDataVertex &st);
            // Removes the vertex with the given index
            bool removeVertex (unsigned int vIndex);
            // Adds the edge data between the given vertex IDs.  Success is returned.
            bool addEdge (unsigned int v1, unsigned int v2, double weight=1.0,
                          const PlannerDataEdge &edge = base::PlannerDataEdge());
            // Adds the edge data between the given vertex data points.  The 
            // vertices are added to the data if they are not already in the 
            // structure.  Success is returned.
            bool addEdge (const PlannerDataVertex &v1, const PlannerDataVertex &v2, 
                          double weight=1.0, const PlannerDataEdge &edge = PlannerDataEdge());
            // Removes the edge between vertex IDs v1 and v2
            bool removeEdge (unsigned int v1, unsigned int v2);
            // Removes the edge between the vertices associated with the given vertex data
            bool removeEdge (const PlannerDataVertex &v1, const PlannerDataVertex &v2);
            // Set the tag associated with the given state
            bool tagState (const base::State* st, int tag);

            // Extract a Boost.Graph object from this PlannerData.  Use of this
            // method requires inclusion of PlannerDataGraph.h
            Graph& toBoostGraph(void);

            /** \brief Any extra properties (key-value pairs) the planner can set. */
            std::map<std::string, std::string> properties;
            
        private:
            // Abstract pointer that points to the Boost.Graph structure.
            // Obscured to prevent unnecessary inclusion of BGL throughout the 
            // rest of the code.
            void* graph;
        };
    }
}

#endif
