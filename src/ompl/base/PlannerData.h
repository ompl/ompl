/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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
#include <map>
#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>

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
            /// \brief Constructor.  Takes a state pointer and an optional integer tag.
            PlannerDataVertex (const base::State* st, int tag = 0) : state_(st), tag_(tag) {}
            /// \brief Copy constructor.
            PlannerDataVertex (const PlannerDataVertex& rhs) : state_(rhs.state_), tag_(rhs.tag_) {}
            virtual ~PlannerDataVertex (void) {}

            /// \brief Returns the integer tag associated with this vertex.
            virtual int  getTag (void) const { return tag_; }
            /// \brief Set the integer tag associated with this vertex.
            virtual void setTag (int tag) { tag_ = tag; }
            /// \brief Retrieve the state associated with this vertex.
            virtual const base::State* getState(void) const { return state_; }

            /// \brief Return a clone of this object, allocated from the heap.
            virtual PlannerDataVertex* clone (void) const
            {
                return new PlannerDataVertex(*this);
            }

            /// \brief Equivalence operator.  Return true if the state pointers are equal.
            virtual bool operator == (const PlannerDataVertex &rhs) const
            {
                // States should be unique
                return state_ == rhs.state_;
            }

            /// \brief Returns true if this vertex is not equal to the argument.
            /// This is the complement of the == operator.
            bool operator != (const PlannerDataVertex &rhs) const
            {
                return !(*this == rhs);
            }

        protected:
            /// \brief The state represented by this vertex
            const base::State* state_;
            /// \brief A generic integer tag for this state.  Not used for equivalence checking.
            int tag_;
        };

        /// \brief Base class for a PlannerData edge.
        class PlannerDataEdge
        {
        public:
            PlannerDataEdge (void) {}
            virtual ~PlannerDataEdge (void) {}
            /// \brief Return a clone of this object, allocated from the heap.
            virtual PlannerDataEdge* clone () const { return new PlannerDataEdge(); }

            /// \brief Returns true if the edges point to the same memory
            virtual bool operator == (const PlannerDataEdge &rhs) const
            {
                return this == &rhs;
            }

            /// \brief Returns true if the edges do not point to the same memory.
            /// This is the complement of the == operator.
            bool operator != (const PlannerDataEdge &rhs) const
            {
                return !(*this == rhs);
            }
        };

        /// @cond IGNORE
        ClassForward(PlannerData);
        /// @endcond

        /// \brief Object containing planner generated vertex and edge data.  It
        /// is assumed that all vertices are unique, and only a single directed
        /// edge connects two vertices.
        class PlannerData : boost::noncopyable
        {
        public:
            class Graph;
            /// \brief A function that accepts two vertex data sets and the edge data, and returns the weight of the edge
            typedef boost::function<double (const PlannerDataVertex&, const PlannerDataVertex&, const PlannerDataEdge&)> EdgeWeightFn;

            /// \brief Representation for a non-existant edge
            static const PlannerDataEdge   NO_EDGE;
            /// \brief Representation for a non-existant vertex
            static const PlannerDataVertex NO_VERTEX;
            /// \brief Representation of an invalid edge weight
            static const double            INVALID_WEIGHT;
            /// \brief Representation of an invalid vertex index
            static const unsigned int      INVALID_INDEX;

            PlannerData(void);
            virtual ~PlannerData(void);

            /// \name PlannerData construction
            /// \{

            /// \brief Adds the given vertex to the graph data.  The vertex index
            /// is returned.  Duplicates are not added.  If a vertex is duplicated,
            /// the index of the existing vertex is returned instead.
            /// Indexes are volatile and may change after adding/removing a subsequent vertex.
            unsigned int addVertex (const PlannerDataVertex &st);
            /// \brief Adds the given vertex to the graph data, and marks it as a
            /// start vertex.  The vertex index is returned.  Duplicates are not added.
            /// If a vertex is duplicated, the index of the existing vertex is returned instead.
            /// Indexes are volatile and may change after adding/removing a subsequent vertex.
            unsigned int addStartVertex (const PlannerDataVertex &v);
            /// \brief Adds the given vertex to the graph data, and marks it as a
            /// start vertex.  The vertex index is returned.  Duplicates are not added.
            /// If a vertex is duplicated, the index of the existing vertex is returned instead.
            /// Indexes are volatile and may change after adding/removing a subsequent vertex.
            unsigned int addGoalVertex  (const PlannerDataVertex &v);
            /// \brief Mark the given state as a start vertex.  If the given state does not exist in a
            /// vertex, false is returned.
            bool markStartState (const base::State* st);
            /// \brief Mark the given state as a goal vertex.  If the given state does not exist in a
            /// vertex, false is returned.
            bool markGoalState (const base::State* st);
            /// \brief Set the integer tag associated with the given state.  If the given
            /// state does not exist in a vertex, false is returned.
            bool tagState (const base::State* st, int tag);
            /// \brief Removes the vertex associated with the given data.  If the
            /// vertex does not exist, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            bool removeVertex (const PlannerDataVertex &st);
            /// \brief Removes the vertex with the given index.  If the index is
            /// out of range, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            bool removeVertex (unsigned int vIndex);
            /// \brief Adds a directed edge between the given vertex indexes.  An optional
            /// edge structure and weight can be supplied.  Success is returned.
            bool addEdge (unsigned int v1, unsigned int v2,
                          const PlannerDataEdge &edge = PlannerDataEdge(), double weight=1.0);
            /// \brief Adds a directed edge between the given vertex indexes.  The
            /// vertices are added to the data if they are not already in the
            /// structure.  An optional edge structure and weight can also be supplied.
            /// Success is returned.
            bool addEdge (const PlannerDataVertex &v1, const PlannerDataVertex &v2,
                          const PlannerDataEdge &edge = PlannerDataEdge(), double weight=1.0);
            /// \brief Removes the edge between vertex indexes \e v1 and \e v2.  Success is returned.
            bool removeEdge (unsigned int v1, unsigned int v2);
            /// \brief Removes the edge between the vertices associated with the given vertex data.
            /// Success is returned.
            bool removeEdge (const PlannerDataVertex &v1, const PlannerDataVertex &v2);
            /// \brief Clears the entire data structure
            void clear (void);

            /// \}
            /// \name PlannerData Properties
            /// \{

            /// \brief Retrieve the number of edges in this structure
            unsigned int numEdges (void) const;
            /// \brief Retrieve the number of vertices in this structure
            unsigned int numVertices (void) const;
            /// \brief Returns the number of start vertices
            unsigned int numStartVertices (void) const;
            /// \brief Returns the number of goal vertices
            unsigned int numGoalVertices (void) const;

            /// \}
            /// \name PlannerData vertex lookup
            /// \{

            /// \brief Check whether a vertex exists with the given vertex data
            bool vertexExists (const PlannerDataVertex &v) const;
            /// \brief Retrieve a reference to the vertex object with the given
            /// index.  If this vertex does not exist, NO_VERTEX is returned.
            const PlannerDataVertex& getVertex (unsigned int index) const;
            /// \brief Retrieve a reference to the vertex object with the given
            /// index.  If this vertex does not exist, NO_VERTEX is returned.
            PlannerDataVertex& getVertex (unsigned int index);
            /// \brief Retrieve a reference to the ith start vertex object.  If
            /// \e i is greater than the number of start vertices, NO_VERTEX is returned.
            const PlannerDataVertex& getStartVertex (unsigned int i) const;
            /// \brief Retrieve a reference to the ith start vertex object.  If
            /// \e i is greater than the number of start vertices, NO_VERTEX is returned.
            PlannerDataVertex& getStartVertex (unsigned int i);
            /// \brief Retrieve a reference to the ith goal vertex object.  If
            /// \e i is greater than the number of goal vertices, NO_VERTEX is returned.
            const PlannerDataVertex& getGoalVertex (unsigned int i) const;
            /// \brief Retrieve a reference to the ith goal vertex object.  If
            /// \e i is greater than the number of goal vertices, NO_VERTEX is returned.
            PlannerDataVertex& getGoalVertex (unsigned int i);
            /// \brief Returns the index of the ith start state.
            /// INVALID_INDEX is returned if \e i is out of range.
            /// Indexes are volatile and may change after adding/removing a vertex.
            unsigned int getStartIndex (unsigned int i) const;
            /// \brief Returns the index of the ith goal state.
            /// INVALID_INDEX is returned if \e i is out of range
            /// Indexes are volatile and may change after adding/removing a vertex.
            unsigned int getGoalIndex  (unsigned int i) const;
            /// \brief Returns true if the given vertex index is marked as a start vertex
            bool isStartVertex (unsigned int index) const;
            /// \brief Returns true if the given vertex index is marked as a goal vertex
            bool isGoalVertex (unsigned int index) const;
            /// \brief Return the index for the vertex associated with the given data.
            /// INVALID_INDEX is returned if this vertex does not exist.
            /// Indexes are volatile and may change after adding/removing a vertex.
            unsigned int vertexIndex (const PlannerDataVertex &v) const;

            /// \}
            /// \name PlannerData edge lookup
            /// \{

            /// \brief Check whether an edge between vertex index \e v1 and index \e v2 exists
            bool edgeExists (unsigned int v1, unsigned int v2) const;
            /// \brief Retrieve a reference to the edge object connecting vertices
            /// with indexes \e v1 and \e v2. If this edge does not exist, NO_EDGE is returned.
            const PlannerDataEdge& getEdge (unsigned int v1, unsigned int v2) const;
            /// \brief Retrieve a reference to the edge object connecting vertices
            /// with indexes \e v1 and \e v2. If this edge does not exist, NO_EDGE is returned.
            PlannerDataEdge& getEdge (unsigned int v1, unsigned int v2);
            /// \brief Returns a list of the vertex indexes directly connected to
            /// vertex with index \e v.  The number of outgoing edges from \e v is returned
            unsigned int getEdges (unsigned int v, std::vector<unsigned int>& edgeList) const;
            /// \brief Returns a map of out-going edges from vertex with index \e v.
            /// Key = vertex index, value = edge structure.  The number of outgoing edges from \e v is returned
            unsigned int getEdges (unsigned int v, std::map<unsigned int, const PlannerDataEdge*> &edgeMap) const;
            /// \brief Returns the weight of the edge between the given vertex indexes.
            /// INVALID_WEIGHT is returned for a non-existant edge.
            double getEdgeWeight (unsigned int v1, unsigned int v2) const;
            /// \brief Sets the weight of the edge between the given vertex indexess.
            /// If an edge between v1 and v2 does not exist, false is returned.
            bool setEdgeWeight (unsigned int v1, unsigned int v2, double weight);
            /// \brief Computes the weight for all edges given the EdgeWeightFn \e f
            void computeEdgeWeights(const EdgeWeightFn& f);

            /// \}
            /// \name Output methods
            /// \{

            /// \brief Writes a Graphviz dot file of this structure to the given stream
            void printGraphviz (std::ostream& out = std::cout) const;
            /// \brief Writes a GraphML file of this structure to the given stream
            void printGraphML(std::ostream& out = std::cout) const;

            /// \}
            /// \name Advanced graph extraction
            /// \{

            /// \brief Extract a Boost.Graph object from this PlannerData.
            /// \remarks Use of this method requires inclusion of PlannerDataGraph.h  The
            /// graph returned can safely be used to inspect the structure or add vertices
            /// and edges.  Removal of vertices and edges should use the
            /// PlannerData::removeVertex and PlannerData::removeEdge methods to ensure
            /// proper memory clean-up.
            Graph& toBoostGraph (void);
            /// \brief Extract a Boost.Graph object from this PlannerData.
            /// \remarks Use of this method requires inclusion of PlannerDataGraph.h  The
            /// graph returned can safely be used to inspect the structure or add vertices
            /// and edges.  Removal of vertices and edges should use the
            /// PlannerData::removeVertex and PlannerData::removeEdge methods to ensure
            /// proper memory clean-up.
            const Graph& toBoostGraph (void) const;

            /// \}

            /// \brief Any extra properties (key-value pairs) the planner can set.
            std::map<std::string, std::string>   properties;

        protected:
            /// \brief A mapping of states to vertex indexes.  For fast lookup of vertex index.
            std::map<const State*, unsigned int> stateIndexMap_;
            /// \brief A mutable listing of the vertices marked as start states.  Stored in sorted order.
            std::vector<unsigned int>            startVertexIndices_;
            /// \brief A mutable listing of the vertices marked as goal states.  Stored in sorted order.
            std::vector<unsigned int>            goalVertexIndices_;

        private:
            // Abstract pointer that points to the Boost.Graph structure.
            // Obscured to prevent unnecessary inclusion of BGL throughout the
            // rest of the code.
            void* graphRaw_;
        };
    }
}

#endif
