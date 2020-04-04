/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_PLANNER_
#define OMPL_BASE_PLANNER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/GenericParam.h"
#include "ompl/util/Console.h"
#include "ompl/util/Time.h"
#include "ompl/util/ClassForward.h"
#include <functional>
#include <boost/concept_check.hpp>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Planner */
        OMPL_CLASS_FORWARD(Planner);
        /// @endcond

        /** \class ompl::base::PlannerPtr
            \brief A shared pointer wrapper for ompl::base::Planner */

        /** \brief Helper class to extract valid start & goal
            states. Usually used internally by planners.

            This class is meant to behave correctly if the user
            updates the problem definition between subsequent calls to
            ompl::base::Planner::solve() \b without calling
            ompl::base::Planner::clear() in between. Only allowed
            changes to the problem definition are accounted for:
            adding of starring states or adding of goal states for
            instances inherited from
            ompl::base::GoalSampleableRegion. */
        class PlannerInputStates
        {
        public:
            /** \brief Default constructor. No work is performed. */
            PlannerInputStates(const PlannerPtr &planner) : planner_(planner.get())
            {
                tempState_ = nullptr;
                update();
            }

            /** \brief Default constructor. No work is performed. */
            PlannerInputStates(const Planner *planner) : planner_(planner)
            {
                tempState_ = nullptr;
                update();
            }

            /** \brief Default constructor. No work is performed. A
                call to use() needs to be made, before making any
                calls to nextStart() or nextGoal(). */
            PlannerInputStates()
            {
                tempState_ = nullptr;
                clear();
            }

            /** \brief Destructor. Clear allocated memory. */
            ~PlannerInputStates()
            {
                clear();
            }

            /** \brief Clear all stored information. */
            void clear();

            /** \brief Forget how many states were returned by
                nextStart() and nextGoal() and return all states
                again */
            void restart();

            /** \brief Set the space information and problem
                definition this class operates on, based on the
                available planner instance. Returns true if changes
                were found (different problem definition) and clear()
                was called. */
            bool update();

            /** \brief Set the problem definition this class operates on.
                If a planner is not set in the constructor argument, a call
                to this function is needed before any calls to nextStart()
                or nextGoal() are made. Returns true if changes were found
                (different problem definition) and clear() was called. */
            bool use(const ProblemDefinitionPtr &pdef);

            /** \brief Set the problem definition this class operates on.
                If a planner is not set in the constructor argument, a call
                to this function is needed before any calls to nextStart()
                or nextGoal() are made. Returns true if changes were found
                (different problem definition) and clear() was called. */
            bool use(const ProblemDefinition *pdef);

            /** \brief Check if the problem definition was set, start
                state are available and goal was set */
            void checkValidity() const;

            /** \brief Return the next valid start state or nullptr if no
                more valid start states are available. */
            const State *nextStart();

            /** \brief Return the next valid goal state or nullptr if no
                more valid goal states are available.  Because
                sampling of goal states may also produce invalid
                goals, this function takes an argument that specifies
                whether a termination condition has been reached.  If
                the termination condition evaluates to true the
                function terminates even if no valid goal has been
                found. */
            const State *nextGoal(const PlannerTerminationCondition &ptc);

            /** \brief Same as above but only one attempt is made to find a valid goal. */
            const State *nextGoal();

            /** \brief Check if there are more potential start states */
            bool haveMoreStartStates() const;

            /** \brief Check if there are more potential goal states */
            bool haveMoreGoalStates() const;

            /** \brief Get the number of start states from the problem
                definition that were already seen, including invalid
                ones. */
            unsigned int getSeenStartStatesCount() const
            {
                return addedStartStates_;
            }

            /** \brief Get the number of sampled goal states, including invalid ones */
            unsigned int getSampledGoalsCount() const
            {
                return sampledGoalsCount_;
            }

        private:
            const Planner *planner_{nullptr};

            unsigned int addedStartStates_;
            unsigned int sampledGoalsCount_;
            State *tempState_;

            const ProblemDefinition *pdef_;
            const SpaceInformation *si_;
        };

        /** \brief Properties that planners may have */
        struct PlannerSpecs
        {
            PlannerSpecs() = default;

            /** \brief The type of goal specification the planner can use */
            GoalType recognizedGoal{GOAL_ANY};

            /** \brief Flag indicating whether multiple threads are used in the computation of the planner */
            bool multithreaded{false};

            /** \brief Flag indicating whether the planner is able to compute approximate solutions */
            bool approximateSolutions{false};

            /** \brief Flag indicating whether the planner attempts to optimize the path and reduce its length until the
                maximum path length specified by the goal representation is satisfied */
            bool optimizingPaths{false};

            /** \brief Flag indicating whether the planner is able to account for the fact that the validity of a motion
               from A to B may not be the same as the validity of a motion from B to A.
                If this flag is true, the planner will return solutions that do not make this assumption. Usually
               roadmap-based planners make this assumption and tree-based planners do not. */
            bool directed{false};

            /** \brief Flag indicating whether the planner is able to prove that no solution path exists. */
            bool provingSolutionNonExistence{false};

            /** \brief Flag indicating whether the planner is able to report the computation of intermediate paths. */
            bool canReportIntermediateSolutions{false};
        };

        /** \brief Base class for a planner */
        class Planner
        {
        public:
            // non-copyable
            Planner(const Planner &) = delete;
            Planner &operator=(const Planner &) = delete;

            /** \brief Constructor */
            Planner(SpaceInformationPtr si, std::string name);

            /** \brief Destructor */
            virtual ~Planner() = default;

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are casting to is indeed a planner */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Planner *>));

                return static_cast<T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are casting to is indeed a Planner */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Planner *>));

                return static_cast<const T *>(this);
            }

            /** \brief Get the space information this planner is using */
            const SpaceInformationPtr &getSpaceInformation() const;

            /** \brief Get the problem definition the planner is trying to solve */
            const ProblemDefinitionPtr &getProblemDefinition() const;

            /** \brief Get the problem definition the planner is trying to solve */
            ProblemDefinitionPtr &getProblemDefinition();

            /** \brief Get the planner input states */
            const PlannerInputStates &getPlannerInputStates() const;

            /** \brief Set the problem definition for the planner. The
                problem needs to be set before calling solve(). Note:
                If this problem definition replaces a previous one, it
                may also be necessary to call clear() or clearQuery(). */
            virtual void setProblemDefinition(const ProblemDefinitionPtr &pdef);

            /** \brief Function that can solve the motion planning problem. This
                function can be called multiple times on the same problem,
                without calling clear() in between. This allows the planner to
                continue work for more time on an unsolved problem, for example.
                If this option is used, it is assumed the problem definition is
                not changed (unpredictable results otherwise). The only change
                in the problem definition that is accounted for is the addition
                of starting or goal states (but not changing previously added
                start/goal states). If clearQuery() is called, the planner may
                retain prior datastructures generated from a previous query on a
                new problem definition. The function terminates if the call to
                \e ptc returns true. */
            virtual PlannerStatus solve(const PlannerTerminationCondition &ptc) = 0;

            /** \brief Same as above except the termination condition
                is only evaluated at a specified interval. */
            PlannerStatus solve(const PlannerTerminationConditionFn &ptc, double checkInterval);

            /** \brief Same as above except the termination condition
                is solely a time limit: the number of seconds the
                algorithm is allowed to spend planning. */
            PlannerStatus solve(double solveTime);

            /** \brief Clear all internal datastructures. Planner
                settings are not affected. Subsequent calls to solve()
                will ignore all previous work. */
            virtual void clear();

            /** \brief Clears internal datastructures of any query-specific
                information from the previous query. Planner settings are not
                affected. The planner, if able, should retain all datastructures
                generated from previous queries that can be used to help solve
                the next query. Note that clear() should also clear all
                query-specific information along with all other datastructures
                in the planner. By default clearQuery() calls clear(). */
            virtual void clearQuery();

            /** \brief Get information about the current run of the
                motion planner. Repeated calls to this function will
                update \e data (only additions are made). This is
                useful to see what changed in the exploration
                datastructure, between calls to solve(), for example
                (without calling clear() in between).  */
            virtual void getPlannerData(PlannerData &data) const;

            /** \brief Get the name of the planner */
            const std::string &getName() const;

            /** \brief Set the name of the planner */
            void setName(const std::string &name);

            /** \brief Return the specifications (capabilities of this planner) */
            const PlannerSpecs &getSpecs() const;

            /** \brief Perform extra configuration steps, if
                needed. This call will also issue a call to
                ompl::base::SpaceInformation::setup() if needed. This
                must be called before solving */
            virtual void setup();

            /** \brief Check to see if the planner is in a working
                state (setup has been called, a goal was set, the
                input states seem to be in order). In case of error,
                this function throws an exception.*/
            virtual void checkValidity();

            /** \brief Check if setup() was called for this planner */
            bool isSetup() const;

            /** \brief Get the  parameters for this planner */
            ParamSet &params()
            {
                return params_;
            }

            /** \brief Get the  parameters for this planner */
            const ParamSet &params() const
            {
                return params_;
            }

            /** \brief Definition of a function which returns a property about the planner's progress that can be
             * queried by a benchmarking routine */
            using PlannerProgressProperty = std::function<std::string()>;

            /** \brief A dictionary which maps the name of a progress property to the function to be used for querying
             * that property */
            using PlannerProgressProperties = std::map<std::string, PlannerProgressProperty>;

            /** \brief Retrieve a planner's planner progress property map */
            const PlannerProgressProperties &getPlannerProgressProperties() const
            {
                return plannerProgressProperties_;
            }

            /** \brief Print properties of the motion planner */
            virtual void printProperties(std::ostream &out) const;

            /** \brief Print information about the motion planner's settings */
            virtual void printSettings(std::ostream &out) const;

        protected:
            /** \brief This function declares a parameter for this planner instance, and specifies the setter and getter
             * functions. */
            template <typename T, typename PlannerType, typename SetterType, typename GetterType>
            void declareParam(const std::string &name, const PlannerType &planner, const SetterType &setter,
                              const GetterType &getter, const std::string &rangeSuggestion = "")
            {
                params_.declareParam<T>(name,
                                        [planner, setter](T param)
                                        {
                                            (*planner.*setter)(param);
                                        },
                                        [planner, getter]
                                        {
                                            return (*planner.*getter)();
                                        });
                if (!rangeSuggestion.empty())
                    params_[name].setRangeSuggestion(rangeSuggestion);
            }

            /** \brief This function declares a parameter for this planner instance, and specifies the setter function.
             */
            template <typename T, typename PlannerType, typename SetterType>
            void declareParam(const std::string &name, const PlannerType &planner, const SetterType &setter,
                              const std::string &rangeSuggestion = "")
            {
                params_.declareParam<T>(name, [planner, setter](T param)
                                        {
                                            (*planner.*setter)(param);
                                        });
                if (!rangeSuggestion.empty())
                    params_[name].setRangeSuggestion(rangeSuggestion);
            }

            /** \brief Add a planner progress property called \e progressPropertyName with a property querying function
             * \e prop to this planner's progress property map */
            void addPlannerProgressProperty(const std::string &progressPropertyName,
                                            const PlannerProgressProperty &prop)
            {
                plannerProgressProperties_[progressPropertyName] = prop;
            }

            /** \brief The space information for which planning is done */
            SpaceInformationPtr si_;

            /** \brief The user set problem definition */
            ProblemDefinitionPtr pdef_;

            /** \brief Utility class to extract valid input states  */
            PlannerInputStates pis_;

            /** \brief The name of this planner */
            std::string name_;

            /** \brief The specifications of the planner (its capabilities) */
            PlannerSpecs specs_;

            /** \brief A map from parameter names to parameter instances for this planner. This field is populated by
             * the declareParam() function */
            ParamSet params_;

            /** \brief A mapping between this planner's progress property names and the functions used for querying
             * those progress properties */
            PlannerProgressProperties plannerProgressProperties_;

            /** \brief Flag indicating whether setup() has been called */
            bool setup_;
        };

        /** \brief Definition of a function that can allocate a planner */
        using PlannerAllocator = std::function<PlannerPtr(const SpaceInformationPtr &)>;
    }
}

#endif
