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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_LTL_LTLPLANNER_
#define OMPL_CONTROL_PLANNERS_LTL_LTLPLANNER_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/ltl/ProductGraph.h"
#include "ompl/control/planners/ltl/LTLSpaceInformation.h"
#include "ompl/datastructures/PDF.h"
#include <unordered_map>
#include <map>
#include <vector>

namespace ompl
{
    namespace control
    {
        /** \anchor cLTLPlanner
            \brief A planner for generating system trajectories to satisfy
            a logical specification given by an automaton, the propositions
            of which are defined over a decomposition of the system's state space.

            \todo cite papers */
        class LTLPlanner : public base::Planner
        {
        public:
            /** \brief Create an LTLPlanner with a given space and product graph.
                Accepts an optional third parameter to control how much time is spent
                promoting low-level tree exploration along a given high-level lead. */
            LTLPlanner(const LTLSpaceInformationPtr &si, ProductGraphPtr a, double exploreTime = 0.5);

            /** \brief Clears all memory belonging to this LTLPlanner .*/
            ~LTLPlanner() override;

            /// @name ompl::base::Planner Interface
            /// @{

            /** \brief Initializes LTLPlanner data structures. */
            void setup() override;

            /** \brief Clears all datastructures belonging to this LTLPlanner. */
            void clear() override;

            /** \brief Continues solving until a solution is found
                or a given planner termination condition is met.
                Returns true if a solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            /// @}

            /** \brief Helper debug method to access this planner's
                underlying tree of states. */
            void getTree(std::vector<base::State *> &tree) const;

            /** \brief Helper debug method to return the sequence of high-level product
                graph states corresponding to a sequence of low-level continous system states,
                beginning from an optional initial high-level state. */
            std::vector<ProductGraph::State *> getHighLevelPath(const std::vector<base::State *> &path,
                                                                ProductGraph::State *start = nullptr) const;

        protected:
            /** \brief Representation of a motion

                A motion contains pointers to its state, its parent motion, and the control
                that was applied to get from its parent to its state. */
            struct Motion
            {
            public:
                /** \brief Default constructor for Motion. */
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control,
                    given a space. */
                Motion(const SpaceInformation *si);

                /** \brief Motion destructor does not clear memory.
                    Deletions should be performed by the LTLPlanner. */
                virtual ~Motion();

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The parent motion in the tree */
                Motion *parent{nullptr};

                /** \brief The number of steps for which the control is applied */
                unsigned int steps{0};

                /** \brief The high-level state to which this motion belongs */
                ProductGraph::State *abstractState{nullptr};
            };

            /** \brief A structure to hold measurement information for a high-level state,
                as well as the set of tree motions belonging to that high-level state.
                Exactly one ProductGraphStateInfo will exist for each ProductGraph::State. */
            struct ProductGraphStateInfo
            {
                /** \brief Creates an info object with no measurements and no tree motions. */
                ProductGraphStateInfo() = default;

                /** \brief Adds a tree motion to an info object.
                    This method is called whenever a new tree motion is created
                    in the high-level state corresponding to this info object. */
                void addMotion(Motion *m);

                double weight{0.};
                PDF<Motion *> motions;
                std::unordered_map<Motion *, PDF<Motion *>::Element *> motionElems;
                double volume{0.};
                double autWeight{0.};
                unsigned int numSel{0};
                PDF<ProductGraph::State *>::Element *pdfElem{nullptr};
            };

            /** \brief Updates and returns the weight of an abstraction state. */
            virtual double updateWeight(ProductGraph::State *as);

            /** \brief Initializes the info object for a new high-level state. */
            virtual void initAbstractInfo(ProductGraph::State *as);

            /** \brief Compute a set of high-level states along a lead
                to be considered for expansion. */
            virtual void buildAvail(const std::vector<ProductGraph::State *> &lead);

            /** \brief Expand the tree of motions along a given lead
                for a given duration of time.
                Returns true if a solution was found, in which case the endpoint
                of the solution trajectory will be stored in the given Motion pointer.
                Otherwise, returns false. */
            virtual bool explore(const std::vector<ProductGraph::State *> &lead, Motion *&soln, double duration);

            /** \brief Returns the weight of an edge between two given high-level states,
                which we compute as the product of the reciprocals of the weights
                of the two states. */
            virtual double abstractEdgeWeight(ProductGraph::State *a, ProductGraph::State *b) const;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            ControlSamplerPtr controlSampler_;

            /** \brief Handle to the control::SpaceInformation object */
            const LTLSpaceInformation *ltlsi_;

            /** \brief The high level abstaction used to grow the tree structure */
            ProductGraphPtr abstraction_;

            /** \brief Used to sample nonempty regions in which to promote expansion. */
            PDF<ProductGraph::State *> availDist_;

            /** \brief A random number generator. */
            RNG rng_;

            /** \brief Set of all motions. */
            std::vector<Motion *> motions_;

            /** \brief Start state in product graph. */
            ProductGraph::State *prodStart_{nullptr};

            /** \brief Time to spend exploring each lead. */
            double exploreTime_;

            /** \brief Map of abstraction states to their details. */
            std::unordered_map<ProductGraph::State *, ProductGraphStateInfo> abstractInfo_;

        private:
            /** \brief Clears this planner's underlying tree of system states. */
            void clearMotions();
        };
    }
}

#endif
