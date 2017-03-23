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

#include "ompl/base/PlannerDataStorage.h"
#include "ompl/control/PlannerData.h"
#include "ompl/control/SpaceInformation.h"

namespace ompl
{
    namespace control
    {
        /// \copydoc ompl::base::PlannerDataStorage
        /// \brief It is assumed that the edges in stored/loaded PlannerData can
        /// be cast to PlannerDataEdgeControl in this class.  If this is not the
        /// case, see ompl::base::PlannerDataStorage.
        class PlannerDataStorage : public base::PlannerDataStorage
        {
        public:
            /// \brief Default constructor
            PlannerDataStorage() = default;
            /// \brief Destructor
            ~PlannerDataStorage() override = default;

            /// \brief Load the PlannerData structure from the given filename.
            void load(const char *filename, base::PlannerData &pd) override;

            /// \brief Deserializes the structure from the given stream.
            void load(std::istream &in, base::PlannerData &pd) override;

            /// \brief Store (serialize) the structure to the given filename.
            /// The StateSpace and ControlSpace that was used to store the data
            /// must match those inside of the argument PlannerData.
            void store(const base::PlannerData &pd, const char *filename) override;

            /// \brief Load the PlannerData structure from the given stream.
            /// The StateSpace and ControlSpace that was used to store the data
            /// must match those inside of the argument PlannerData.
            void store(const base::PlannerData &pd, std::ostream &out) override;

        protected:
            /// @cond IGNORE
            // Information stored at the beginning of the PlannerData archive
            struct Header : base::PlannerDataStorage::Header
            {
                /// \brief Signature of control space that allocated the saved states in the vertices (see
                /// ompl::base::StateSpace::computeSignature()) */
                std::vector<int> control_signature;

                /// \brief boost::serialization routine
                template <typename Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &boost::serialization::base_object<base::PlannerDataStorage::Header>(*this);
                    ar &control_signature;
                }
            };

            // The object containing all control edge data that will be stored
            struct PlannerDataEdgeControlData : base::PlannerDataStorage::PlannerDataEdgeData
            {
                template <typename Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &boost::serialization::base_object<base::PlannerDataStorage::PlannerDataEdgeData>(*this);
                    ar &control_;
                }

                std::vector<unsigned char> control_;
            };
            /// \endcond

            /// \brief Read \e numEdges from the binary input \e ia and store them as PlannerData.  It is assumed
            /// that the edges can be cast to ompl::control::PlannerDataEdgeControl.
            void loadEdges(base::PlannerData &pd, unsigned int numEdges, boost::archive::binary_iarchive &ia) override
            {
                OMPL_DEBUG("Loading %d PlannerDataEdgeControl objects", numEdges);

                const ControlSpacePtr &space =
                    static_cast<control::PlannerData &>(pd).getSpaceInformation()->getControlSpace();
                std::vector<Control *> controls;

                for (unsigned int i = 0; i < numEdges; ++i)
                {
                    PlannerDataEdgeControlData edgeData;
                    ia >> edgeData;

                    std::vector<unsigned char> ctrlBuf(space->getSerializationLength());
                    Control *ctrl = space->allocControl();
                    controls.push_back(ctrl);
                    space->deserialize(ctrl, &edgeData.control_[0]);
                    const_cast<PlannerDataEdgeControl *>(static_cast<const PlannerDataEdgeControl *>(edgeData.e_))->c_ =
                        ctrl;

                    pd.addEdge(edgeData.endpoints_.first, edgeData.endpoints_.second, *edgeData.e_,
                               base::Cost(edgeData.weight_));

                    // We deserialized the edge object pointer, and we own it.
                    // Since addEdge copies the object, it is safe to free here.
                    delete edgeData.e_;
                }

                // These edges are using control pointers allocated here.
                // To avoid a memory leak, we decouple planner data from the
                // 'planner', which will clone all controls and properly free the
                // memory when PlannerData goes out of scope.  Then it is safe
                // to free all memory allocated here.
                pd.decoupleFromPlanner();

                for (auto &control : controls)
                    space->freeControl(control);
            }

            /// \brief Serialize and store all edges in \e pd to the binary archive.  It is assumed
            /// that the edges can be cast to ompl::control::PlannerDataEdgeControl.
            void storeEdges(const base::PlannerData &pd, boost::archive::binary_oarchive &oa) override
            {
                OMPL_DEBUG("Storing %d PlannerDataEdgeControl objects", pd.numEdges());

                const ControlSpacePtr &space =
                    static_cast<const control::PlannerData &>(pd).getSpaceInformation()->getControlSpace();
                std::vector<unsigned char> ctrl(space->getSerializationLength());

                std::vector<unsigned int> edgeList;
                for (unsigned int fromVertex = 0; fromVertex < pd.numVertices(); ++fromVertex)
                {
                    edgeList.clear();
                    pd.getEdges(fromVertex, edgeList);  // returns the id of each edge

                    // Process edges
                    for (unsigned int toVertex : edgeList)
                    {
                        // Get cost
                        base::Cost weight;
                        if (!pd.getEdgeWeight(fromVertex, toVertex, &weight))
                            OMPL_ERROR("Unable to get edge weight");

                        // Convert to new structure
                        PlannerDataEdgeControlData edgeData;
                        edgeData.e_ = &pd.getEdge(fromVertex, toVertex);
                        edgeData.endpoints_.first = fromVertex;
                        edgeData.endpoints_.second = toVertex;
                        edgeData.weight_ = weight.value();
                        space->serialize(&ctrl[0],
                                         static_cast<const PlannerDataEdgeControl *>(edgeData.e_)->getControl());
                        edgeData.control_ = ctrl;
                        oa << edgeData;

                    }  // for each edge
                }      // for each vertex
            }
        };
    }
}

#endif
