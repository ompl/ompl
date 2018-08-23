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

#ifndef OMPL_BASE_PLANNER_DATA_STORAGE_
#define OMPL_BASE_PLANNER_DATA_STORAGE_

#include "ompl/base/PlannerData.h"
#include "ompl/util/Console.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <fstream>

namespace ompl
{
    namespace base
    {
        /// \brief Object that handles loading/storing a PlannerData object to/from a binary stream.
        /// Serialization of vertices and edges is performed using the Boost archive method
        /// \e serialize.  Derived vertex/edge classes are handled, presuming those classes implement
        /// the \e serialize method.
        /// \remarks Since the \e serialize method for vertices and edges is templated, it cannot
        /// be virtual.  To serialize a derived class AND the base class data, a special call can
        /// be invoked inside of \e serialize that instructs the serializer to also serialize the
        /// base class.  The derived class must also have a GUID exposed to the serializer
        /// for proper deserialization at runtime.  This is performed with the \e BOOST_CLASS_EXPORT
        /// macro.  An example of these items is given below:
        /// \code
        /// #include <boost/serialization/export.hpp>
        ///
        /// class MyVertexClass : public ompl::base::PlannerDataVertex
        /// {
        ///     // ---SNIP---
        ///
        ///     template <class Archive>
        ///     void serialize(Archive & ar, const unsigned int version)
        ///     {
        ///         ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
        ///         // ... (The other members of MyVertexClass)
        ///     }
        /// };
        ///
        /// BOOST_CLASS_EXPORT(MyVertexClass);
        /// \endcode
        class PlannerDataStorage
        {
        public:
            /// \brief Default constructor.
            PlannerDataStorage();
            /// \brief Destructor
            virtual ~PlannerDataStorage();

            /// \brief Store (serialize) the PlannerData structure to the given filename.
            virtual void store(const PlannerData &pd, const char *filename);

            /// \brief Store (serialize) the PlannerData structure to the given stream.
            virtual void store(const PlannerData &pd, std::ostream &out);

            /// \brief Load the PlannerData structure from the given stream.
            /// The StateSpace that was used to store the data must match the
            /// StateSpace inside of the argument PlannerData.
            virtual void load(const char *filename, PlannerData &pd);

            /// \brief Load the PlannerData structure from the given stream.
            /// The StateSpace that was used to store the data must match the
            /// StateSpace inside of the argument PlannerData.
            virtual void load(std::istream &in, PlannerData &pd);

        protected:
            /// \brief Information stored at the beginning of the PlannerData archive
            struct Header
            {
                /// \brief OMPL PlannerData specific marker (fixed value)
                std::uint_fast32_t marker;

                /// \brief Number of vertices stored in the archive
                std::size_t vertex_count;

                /// \brief Number of edges stored in the archive
                std::size_t edge_count;

                /// \brief Signature of state space that allocated the saved states in the vertices (see
                /// ompl::base::StateSpace::computeSignature()) */
                std::vector<int> signature;

                /// \brief boost::serialization routine
                template <typename Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &marker;
                    ar &vertex_count;
                    ar &edge_count;
                    ar &signature;
                }
            };

            /// \brief The object containing all vertex data that will be stored
            struct PlannerDataVertexData
            {
                enum VertexType
                {
                    STANDARD = 0,
                    START,
                    GOAL
                };

                template <typename Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &v_;
                    ar &state_;
                    ar &type_;
                }

                const PlannerDataVertex *v_;
                std::vector<unsigned char> state_;
                VertexType type_;
            };

            /// \brief The object containing all edge data that will be stored
            struct PlannerDataEdgeData
            {
                template <typename Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &e_;
                    ar &endpoints_;
                    ar &weight_;
                }

                const PlannerDataEdge *e_;
                std::pair<unsigned int, unsigned int> endpoints_;
                double weight_;
            };

            /// \brief Read \e numVertices from the binary input \e ia and store them as PlannerData.
            virtual void loadVertices(PlannerData &pd, unsigned int numVertices, boost::archive::binary_iarchive &ia)
            {
                const StateSpacePtr &space = pd.getSpaceInformation()->getStateSpace();
                std::vector<State *> states;
                for (unsigned int i = 0; i < numVertices; ++i)
                {
                    PlannerDataVertexData vertexData;
                    ia >> vertexData;

                    // Deserializing all data in the vertex (except the state)
                    const PlannerDataVertex *v = vertexData.v_;

                    // Allocating a new state and deserializing it from the buffer
                    State *state = space->allocState();
                    states.push_back(state);
                    space->deserialize(state, &vertexData.state_[0]);
                    const_cast<PlannerDataVertex *>(v)->state_ = state;

                    // Record the type of the vertex (i.e. start vertex).
                    if (vertexData.type_ == PlannerDataVertexData::START)
                        pd.addStartVertex(*v);
                    else if (vertexData.type_ == PlannerDataVertexData::GOAL)
                        pd.addGoalVertex(*v);
                    else
                        pd.addVertex(*v);

                    // We deserialized the vertex object pointer, and we own it.
                    // Since addEdge copies the object, it is safe to free here.
                    delete vertexData.v_;
                }

                // These vertices are using state pointers allocated here.
                // To avoid a memory leak, we decouple planner data from the
                // 'planner', which will clone all states and properly free the
                // memory when PlannerData goes out of scope.  Then it is safe
                // to free all memory allocated here.
                pd.decoupleFromPlanner();

                for (auto &state : states)
                    space->freeState(state);
            }

            /// \brief Serialize and store all vertices in \e pd to the binary archive.
            virtual void storeVertices(const PlannerData &pd, boost::archive::binary_oarchive &oa)
            {
                const StateSpacePtr &space = pd.getSpaceInformation()->getStateSpace();
                std::vector<unsigned char> state(space->getSerializationLength());
                for (unsigned int i = 0; i < pd.numVertices(); ++i)
                {
                    PlannerDataVertexData vertexData;

                    // Serializing all data in the vertex (except the state)
                    const PlannerDataVertex &v = pd.getVertex(i);
                    vertexData.v_ = &v;

                    // Record the type of the vertex (i.e. start vertex).
                    if (pd.isStartVertex(i))
                        vertexData.type_ = PlannerDataVertexData::START;
                    else if (pd.isGoalVertex(i))
                        vertexData.type_ = PlannerDataVertexData::GOAL;
                    else
                        vertexData.type_ = PlannerDataVertexData::STANDARD;

                    // Serializing the state contained in this vertex
                    space->serialize(&state[0], v.getState());
                    vertexData.state_ = state;

                    oa << vertexData;
                }
            }

            /// \brief Read \e numEdges from the binary input \e ia and store them as PlannerData.
            virtual void loadEdges(PlannerData &pd, unsigned int numEdges, boost::archive::binary_iarchive &ia)
            {
                for (unsigned int i = 0; i < numEdges; ++i)
                {
                    PlannerDataEdgeData edgeData;
                    ia >> edgeData;
                    pd.addEdge(edgeData.endpoints_.first, edgeData.endpoints_.second, *edgeData.e_,
                               Cost(edgeData.weight_));

                    // We deserialized the edge object pointer, and we own it.
                    // Since addEdge copies the object, it is safe to free here.
                    delete edgeData.e_;
                }
            }

            /// \brief Serialize and store all edges in \e pd to the binary archive.
            virtual void storeEdges(const PlannerData &pd, boost::archive::binary_oarchive &oa)
            {
                std::vector<unsigned int> edgeList;
                for (unsigned int fromVertex = 0; fromVertex < pd.numVertices(); ++fromVertex)
                {
                    edgeList.clear();
                    pd.getEdges(fromVertex, edgeList);  // returns the id of each edge

                    // Process edges
                    for (unsigned int toVertex : edgeList)
                    {
                        // Get cost
                        Cost weight;
                        if (!pd.getEdgeWeight(fromVertex, toVertex, &weight))
                            OMPL_ERROR("Unable to get edge weight");

                        // Convert to new structure
                        PlannerDataEdgeData edgeData;
                        edgeData.e_ = &pd.getEdge(fromVertex, toVertex);
                        edgeData.endpoints_.first = fromVertex;
                        edgeData.endpoints_.second = toVertex;
                        edgeData.weight_ = weight.value();
                        oa << edgeData;

                    }  // for each edge
                }      // for each vertex
            }
        };
    }
}

#endif
