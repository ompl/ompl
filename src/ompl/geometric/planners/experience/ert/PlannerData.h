/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Rice University
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

/* Author: Èric Pairet */

#ifndef OMPL_SEGMENT_PLANNER_DATA_
#define OMPL_SEGMENT_PLANNER_DATA_

#include "ompl/base/PlannerData.h"
#include "ompl/control/SpaceInformation.h"
#include <boost/serialization/base_object.hpp>

namespace ompl
{
    namespace geometric
    {
        namespace ert
        {
            /** \brief Representation of an edge for the ERT planner */
            class PlannerDataEdgeSegment : public base::PlannerDataEdge
            {
            public:
                PlannerDataEdgeSegment(std::vector<base::State*> segment) : segment_(segment)
                {}

                PlannerDataEdgeSegment(const PlannerDataEdgeSegment &rhs) : segment_(rhs.segment_)
                {}

                ~PlannerDataEdgeSegment() override = default;

                base::PlannerDataEdge *clone() const override
                {
                    return static_cast<base::PlannerDataEdge *>(new PlannerDataEdgeSegment(*this));
                }

                std::vector<base::State*> getSegment() const
                {
                    return segment_;
                }

                bool operator==(const PlannerDataEdge &rhs) const override
                {
                const auto *rhsc = static_cast<const PlannerDataEdgeSegment *>(&rhs);
                if (segment_ == rhsc->segment_)
                    return static_cast<const PlannerDataEdge>(*this) == rhs;
                return false;
                }

            protected:
                friend class boost::serialization::access;
                friend class PlannerDataStorage;
                friend class PlannerData;

                PlannerDataEdgeSegment() = default;

                template <class Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &boost::serialization::base_object<base::PlannerDataEdge>(*this);
                }

                std::vector<base::State*> segment_;
            };
        }
    }
}

#endif
