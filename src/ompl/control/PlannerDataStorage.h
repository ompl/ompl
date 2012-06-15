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

#ifndef OMPL_CONTROL_PLANNER_DATA_STORAGE_
#define OMPL_CONTROL_PLANNER_DATA_STORAGE_

#include "ompl/base/PlannerData.h"
#include "ompl/base/StateStorage.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/util/Console.h"
#include <boost/serialization/access.hpp>
#include <fstream>

namespace ompl
{
    namespace control
    {
        class PlannerDataStorage
        {
        public:
            /// \brief Serializes the planner data structure to the given filename.
            static void Serialize(const PlannerData& pd, const char *filename)
            {
                std::ofstream out(filename, std::ios::binary);
                Serialize(pd, out);
                out.close();
            }

            /// \brief Serializes the structure to the given stream.
            static void Serialize(const PlannerData& pd, std::ostream &out)
            {
                const control::SpaceInformationPtr& sic = pd.getSpaceInformation();
                base::StateStorageWithMetadata<PlannerDataMetadata> storage(sic->getStateSpace());

                for (size_t i = 0; i < pd.numVertices(); ++i)
                {
                    // Create a metadata object to store other members of the vertex class
                    PlannerDataMetadata data (&pd.getVertex(i));

                    // Enumerating all edges of this vertex, and storing pointers into the metadata object
                    std::map<unsigned int, const base::PlannerDataEdge*> neighbors;
                    pd.getEdges(i, neighbors);
                    for (std::map<unsigned int, const base::PlannerDataEdge*>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
                    {
                        data.edgeIndexes_.push_back(it->first);
                        data.edgeObjects_.push_back(it->second);

                        const PlannerDataEdgeControl* controlEdge = static_cast<const PlannerDataEdgeControl*>(it->second);
                        if (!controlEdge)
                            throw Exception("Failed to cast edge to PlannerDataEdgeControl");

                        std::vector<unsigned char> control(sic->getControlSpace()->getSerializationLength());
                        sic->getControlSpace()->serialize(&control[0], static_cast<const control::PlannerDataEdgeControl*>(it->second)->getControl());
                        data.controls_.push_back(control);
                    }
                    // Storing the state and all metadata
                    storage.addState(pd.getVertex(i).getState(), data);
                }

                storage.store(out);
            }

            /// \brief Deserializes the structure from the given filename.
            static void Deserialize(const char *filename, PlannerData& pd)
            {
                std::ifstream in(filename, std::ios::binary);
                Deserialize(in, pd);
                in.close();
            }

            /// \brief Deserializes the structure from the given stream.
            static void Deserialize(std::istream &in, PlannerData& pd)
            {
                pd.clear();

                const SpaceInformationPtr& sic = pd.getSpaceInformation();

                base::StateStorageWithMetadata<PlannerDataMetadata> storage(sic->getStateSpace());
                storage.load(in);

                std::vector <Control*> controls;

                // Repopulate the vertices and all associated vertex data
                for (size_t i = 0; i < storage.size(); ++i)
                {
                    PlannerDataMetadata &data = storage.getMetadata(i);
                    // Boost.Serialization automagically knows the derived vertex type.  Cast to the base
                    base::PlannerDataVertex* v = const_cast<base::PlannerDataVertex*>(data.v_);

                    v->state_ = storage.getState(i);
                    pd.addVertex(*v);
                }

                // Loading all edges and edge data
                for (size_t i = 0; i < storage.size(); ++i)
                {
                    PlannerDataMetadata &data = storage.getMetadata(i);  // This is the metadata for vertex i
                    for (size_t j = 0; j < data.edgeIndexes_.size(); ++j) // Iterating over the edges at vertex i
                    {
                        base::PlannerDataEdge* edge = const_cast<base::PlannerDataEdge*>(data.edgeObjects_[j]);
                        PlannerDataEdgeControl *ec = static_cast<PlannerDataEdgeControl*>(edge);

                        if (!ec)
                            throw Exception("Failed to cast edge to PlannerDataEdgeControl");

                        // Allocating a new control to deserialize the raw bytes into
                        Control *c = sic->allocControl();
                        controls.push_back(c);

                        // Deserializing the raw buffer, and setting the edge's control to the new pointer
                        sic->getControlSpace()->deserialize(c, &data.controls_[j][0]);
                        ec->c_ = c;

                        // Adding the edge to the planner data
                        pd.addEdge(i, data.edgeIndexes_[j], *(data.edgeObjects_[j]));
                    }

                    // When pointers in metadata are deserialized, we assume ownership of them.
                    // PlannerData clones all memory for vertices and edges, so it is safe to
                    // free the metadata here.
                    data.freeMemory();
                }

                // We are using pointers from StateStorage in the vertices, which will be
                // freed when storage goes out of scope.  Very important to create a
                // copy of the memory here.
                pd.decoupleFromPlanner();

                // Decoupling clones all controls.  It is safe to free them here.
                for (size_t i = 0; i < controls.size(); ++i)
                    sic->freeControl(controls[i]);
            }

            protected:
                // This class is used internally to serialize vertex and edge
                // data (other than state pointers).
                class PlannerDataMetadata
                {
                public:
                    PlannerDataMetadata() : v_(NULL) {}
                    PlannerDataMetadata(const base::PlannerDataVertex* v) : v_(v) {}

                    void freeMemory()
                    {
                        for (size_t i = 0; i < edgeObjects_.size(); ++i)
                            delete edgeObjects_[i];
                        edgeObjects_.clear();
                        if (v_)
                        {
                            delete v_;
                            v_ = NULL;
                        }
                    }

                    template<typename Archive>
                    void serialize(Archive & ar, const unsigned int version)
                    {
                        ar & v_;
                        ar & edgeIndexes_;
                        ar & edgeObjects_;
                        ar & controls_;
                    }

                    const base::PlannerDataVertex *v_;
                    // ideally these members would constitute a map, but boost.serialize cannot process that structure
                    std::vector<unsigned int> edgeIndexes_;
                    std::vector<const base::PlannerDataEdge*> edgeObjects_;
                    std::vector<std::vector<unsigned char> > controls_;
                };
        };
    }
}

#endif
