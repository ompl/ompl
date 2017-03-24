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

/* Author: Mark Moll */

#ifndef OMPL_CONTROL_PLANNER_DATA_
#define OMPL_CONTROL_PLANNER_DATA_

#include "ompl/base/PlannerData.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/Control.h"
#include <boost/serialization/base_object.hpp>

namespace ompl
{
    namespace control
    {
        /// \brief Representation of an edge in PlannerData for planning with controls.
        /// This structure encodes a specific control and a duration to apply the control.
        /// \remarks If using PlannerDataEdgeControl in conjunction with PlannerDataStorage,
        /// (i.e., storing the PlannerData from a controls planner) you must export a GUID
        /// for PlannerDataEdgeControl so that the serializer can identify the derived
        /// edge class:
        /// \code
        /// #include <boost/serialization/export.hpp>
        /// ...
        /// BOOST_CLASS_EXPORT(ompl::control::PlannerDataEdgeControl);
        /// \endcode
        class PlannerDataEdgeControl : public base::PlannerDataEdge
        {
        public:
            /// \brief Constructor.  Accepts a control pointer and a duration.
            PlannerDataEdgeControl(const Control *c, double duration) : c_(c), duration_(duration)
            {
            }
            /// \brief Copy constructor.
            PlannerDataEdgeControl(const PlannerDataEdgeControl &rhs)
              : c_(rhs.c_), duration_(rhs.duration_)
            {
            }

            ~PlannerDataEdgeControl() override = default;

            base::PlannerDataEdge *clone() const override
            {
                return static_cast<base::PlannerDataEdge *>(new PlannerDataEdgeControl(*this));
            }

            /// \brief Return the control associated with this edge.
            const Control *getControl() const
            {
                return c_;
            }
            /// \brief Return the duration associated with this edge.
            double getDuration() const
            {
                return duration_;
            }

            bool operator==(const PlannerDataEdge &rhs) const override
            {
                const auto *rhsc = static_cast<const PlannerDataEdgeControl *>(&rhs);
                if (c_ == rhsc->c_)
                    return static_cast<const PlannerDataEdge>(*this) == rhs;
                return false;
            }

        protected:
            friend class boost::serialization::access;
            friend class PlannerDataStorage;
            friend class PlannerData;

            PlannerDataEdgeControl() = default;

            template <class Archive>
            void serialize(Archive &ar, const unsigned int /*version*/)
            {
                ar &boost::serialization::base_object<base::PlannerDataEdge>(*this);
                ar &duration_;
                // Serializing the control is handled by control::PlannerDataStorage
            }

            const Control *c_{nullptr};
            double duration_;
        };

        /// \copydoc ompl::base::PlannerData
        /// \brief This class assumes edges are derived from PlannerDataEdgeControl.
        /// If this is not the case, see base::PlannerData.
        class PlannerData : public base::PlannerData
        {
        public:
            /// \brief Constructor.  Accepts a SpaceInformationPtr for the space planned in.
            PlannerData(const SpaceInformationPtr &siC);
            /// \brief Destructor.
            ~PlannerData() override;

            /// \brief Removes the vertex associated with the given data.  If the
            /// vertex does not exist, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            bool removeVertex(const base::PlannerDataVertex &st) override;
            /// \brief Removes the vertex with the given index.  If the index is
            /// out of range, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            bool removeVertex(unsigned int vIndex) override;

            /// \brief Removes the edge between vertex indexes \e v1 and \e v2.  Success is returned.
            bool removeEdge(unsigned int v1, unsigned int v2) override;
            /// \brief Removes the edge between the vertices associated with the given vertex data.
            /// Success is returned.
            bool removeEdge(const base::PlannerDataVertex &v1, const base::PlannerDataVertex &v2) override;

            /// \brief Clears the entire data structure
            void clear() override;

            /// \brief Creates a deep copy of the states contained in the vertices of this
            /// PlannerData structure so that when the planner that created this instance goes
            /// out of scope, all data remains intact.
            /// \remarks Shallow state pointers inside of the PlannerDataVertex objects already
            /// in this PlannerData will be replaced with clones which are scoped to this PlannerData
            /// object.  A subsequent call to this method is necessary after any other vertices are
            /// added to ensure that this PlannerData instance is fully decoupled.
            void decoupleFromPlanner() override;

            /// \brief Return the instance of SpaceInformation used in this PlannerData
            const SpaceInformationPtr &getSpaceInformation() const;

            /// \brief Returns true if this PlannerData instance has controls associated with it
            bool hasControls() const override;

        protected:
            /// \brief The instance of control::SpaceInformation associated with this data
            SpaceInformationPtr siC_;
            /// \brief A list of controls that are allocated during the decoupleFromPlanner method.
            /// These controls are freed by PlannerData in the destructor.
            std::set<Control *> decoupledControls_;

        private:
            void freeMemory();
        };
    }
}

#endif
