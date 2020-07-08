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

#ifndef OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_ERT_
#define OMPL_GEOMETRIC_PLANNERS_EXPERIENCE_ERT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/PDF.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/geometric/planners/experience/planner_data_edge_segment.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gERT
           @par Short description
           ERT is a tree-based motion planner that uses the following
           idea: ERT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           @par External documentation
           J. Kuffner and S.M. LaValle, ERT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */

        /** \brief Rapidly-exploring Random Trees */
        class ERT : public base::Planner
        {
        public:
            /** \brief Constructor */
            ERT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~ERT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            // NOTE: not used!
            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            // NOTE: not used!
            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            // NOTE: not used!
            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            // NOTE: not used!
            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the minimum fraction of the experience to be extracted

                some more description... */
            void setExperienceFractionMin(double segment_fraction_min)
            {
                segment_fraction_min_ = segment_fraction_min;
            }

            /** \brief Get the minimum experience fraction the planner is using */
            double getExperienceFractionMin() const
            {
                return segment_fraction_min_;
            }

            /** \brief Set the maximum fraction of the experience to be extracted

                some more description... */
            void setExperienceFractionMax(double segment_fraction_max)
            {
                segment_fraction_max_ = segment_fraction_max;
            }

            /** \brief Get the maximum experience fraction the planner is using */
            double getExperienceFractionMax() const
            {
                return segment_fraction_max_;
            }

            /** \brief Set the tubular neighbourhood around the experience to delimit the tree expansion

                some more description... */
            void setExperienceTubularRadius(double segment_noise)
            {
                experience_tubular_radius_ = segment_noise;
            }

            /** \brief Get the tubular neighbourhood around the experience the planner is using */
            double getExperienceTubularRadius() const
            {
                return experience_tubular_radius_;
            }

            // NOTE: could the argument be a PathGeometric?
            // NOTE: is there a more elegant way to initialise experience_
            /** \brief Set the experience to exploit by the ERT

                some more description... */
            void setExperience(std::vector<base::State*> experience)
            {
                experience_.resize(experience.size());
                // candidate_segment_.resize(experience.size());
                for (size_t i = 0; i < experience.size(); ++i)
                {
                    base::State *s = si_->allocState();
                    si_->copyState(s, experience[i]);
                    experience_[i] = s;

                    // NOTE: THIS COULD BE cloneState()!

                    // base::State *ss = si_->allocState();
                    // candidate_segment_[i] = ss;
                }

                // experience_.clear();
                // experience_.reserve(experience.size());
                // for (auto &state : experience)
                // {
                //     base::State *s = si_->allocState();
                //     si_->copyState(s, state);
                //     experience_.push_back(s);
                // }

            }

            // NOTE: update according type of experience
            /** \brief Get the experience the planner is using

                Before solve it will return the same experience as provided.
                After solve it will return the updated one, if enabled*/
            std::vector<base::State*> getExperience() const
            {
                return experience_;
            }

            /** \brief Get the experience the planner is using

                Before solve it will return the same experience as provided.
                After solve it will return the updated one, if enabled*/
            PathGeometric getExperienceAsPathGeometric() const
            {
                auto path(std::make_shared<PathGeometric>(si_));
                for (auto &state : experience_)
                    path->append(state);
                return *path;
            }

            /** \brief Specify whether the provided experience is to be updated as its mapping onto the current planning problem

                some more description... */
            void setExperienceInitialUpdate(bool experienceInitialUpdate)
            {
                experienceInitialUpdate_ = experienceInitialUpdate;
            }

            /** \brief Return true if the provided experience is to be updated as its mapping onto the current planning problem */
            bool getExperienceInitialUpdate() const
            {
                return experienceInitialUpdate_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                // NOTE: why not to change the name of this for segment?
                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si, const unsigned int &ps) :
                    state(si->allocState()),
                    phase_span(ps)
                {
                    segment.resize(ps);
                    for (auto &s : segment)
                        s = si->allocState();
                }

                ~Motion() = default;

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                // NOTE: the state and demo_index could be a class defining the ConfigurationPhaseSpace as in the paper
                /** \brief The end state of the motion */
                base::State *state{nullptr};

                // NOTE: the state and demo_index could be a class defining the ConfigurationPhaseSpace as in the paper
                /** \brief The end phase of the motion */
                unsigned int demo_index;

                /** \brief A pointer to the corresponding element in the probability distribution function */
                // about the end state!
                PDF<Motion *>::Element *element{nullptr};

                // NOTE: could this be called cost, to be more generic?
                /** \brief The number of attempts to expand the tree from the end state of this motion */
                unsigned int selected_num{0};

                // NOTE: add option to resample?
                // NOTE: would using this PlannerDataEdgeSegment better?
                // NOTE: would using this PathGeometric better?
                // the difference between PathGeometric and what Segment means for us is that the latter contains uniform samples along the trajectory, whereas PathGeometric does not forcefully need to (although it can be resampled with .interpolate())
                /** \brief All states composing the motion */
                std::vector<base::State*> segment;

                /** \brief The phase span of the motion */
                unsigned int phase_span{0};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

             /** \brief Compute motion weight in the PDF according to the number of times such motion has been picked to expand the tree */
            // NOTE: ideally, this function should be a pointer to a user-defined function
            // that allows to set pdef uniform, or any other metric...
            double weightFunction(const Motion *m)
            {
               return (1. / (100 * m->selected_num + 1));
            }

            // NOTE: to document
            /** \brief  */
            bool isSegmentValid(const std::vector<base::State*> &segment);
            bool isSegmentValid(const Motion *tmotion);

            // NOTE: making it const, does the content const too?
            // NOTE: to document
            /** \brief  */
            // bool getSegment(std::vector<base::State*> &experience, const base::State *istate, const unsigned int &ialpha, base::State *tstate, unsigned int &talpha);
            // void getSegment(const base::State *istate, const unsigned int &ialpha, base::State *tstate, unsigned int &talpha, const bool connect_flag, std::vector<base::State*> &experience);
            void getSegment(const Motion *imotion, Motion *tmotion, const bool connect_flag);

            /** \brief The probability distribution function over states in the tree */
            PDF<Motion *> pdf_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double goalBias_{.05};

            // NOTE: not used!
            /** \brief  */
            double maxDistance_{0.};

            // NOTE: update name following OMPL standars (no underscore), as example
            /** \brief The minimum fraction of the experience to be extracted as micro-experience (segment) */
            // double alphaMinimum_{0.05};
            double segment_fraction_min_{0.05};

            // NOTE: update name following OMPL standars (no underscore), as example
            /** \brief The maximum fraction of the experience to be extracted as micro-experience (segment) */
            // double alphaMaximum_{0.1};
            double segment_fraction_max_{0.1};

            // NOTE: update name following OMPL standars (no underscore), as example
            /** \brief The tubular neighbourhood around the experience

                This parameter indicates the maximum distance ti which ERT will expand.
                Currently, the tubular neighbourhood is symmetric in all dimensions */
            // double epsilon_{5};
            double experience_tubular_radius_{5};

            // NOTE: not used!
            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            // NOTE: should this be a Motion?
            // NOTE: could this be a PathGeometric?
            /** \brief The task-relevant prior experience to exploit for the ERT */
            std::vector<base::State*> experience_;

            // NOTE: should this be a Motion?
            // NOTE: does this need to be documented?
            /** \brief Pre-allocate memory for sampling segments */
            // std::vector<base::State*> candidate_segment_;

            // NOTE: should this be a Motion?
            // NOTE: does this need to be documented?
            /** \brief Indicate last index in candidate_segment_ corresponding to the segment */
            // unsigned int candidate_segment_last_index_{0};
            // unsigned int candidate_segment_phase_span_;

            /** \brief Flag indicating whether the experience is map onto the current planning problem at start */
            bool experienceInitialUpdate_{true};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif
