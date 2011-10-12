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

#ifndef OMPL_BASE_STATE_SPACE_
#define OMPL_BASE_STATE_SPACE_

#include "ompl/base/State.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {

        /** \brief Forward declaration of ompl::base::StateSpace */
        ClassForward(StateSpace);

        /** \class ompl::base::StateSpacePtr
            \brief A boost shared pointer wrapper for ompl::base::StateSpace */


        /** \brief Representation of a space in which planning can be
            performed. Topology specific sampling, interpolation and distance
            are defined.

            See \ref implementingStateSpaces. */
        class StateSpace : private boost::noncopyable
        {
        public:

            /** \brief Define the type of state allocated by this space */
            typedef State StateType;

            /** \brief Constructor. Assigns a @b unique name to the space */
            StateSpace(void);

            virtual ~StateSpace(void);

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as(void)
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as(void) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<const T*>(this);
            }

            /** @name Generic functionality for state spaces
                @{ */

            /** \brief Check if the state space is compound */
            virtual bool isCompound(void) const;

            /** \brief Check if the set of states is discrete

                \note In fact, because of limited numerical precision,
                the representation of all spaces is discrete; this
                function returns true if the corresponding
                mathematical object is a discrete one. */
            virtual bool isDiscrete(void) const;

            /** \brief Check if this is a hybrid state space (i.e., both discrete and continuous components exist)*/
            virtual bool isHybrid(void) const;

            /** \brief Get the name of the state space */
            const std::string& getName(void) const;

            /** \brief Set the name of the state space */
            void setName(const std::string &name);

            /** \brief Get the type of the state space. The type can be
                used to verify whether two space instances are of
                the same type (e.g., SO2) */
            int getType(void) const
            {
                return type_;
            }

            /** \brief Return true if \e other is a space included (perhaps equal, perhaps a subspace) in this one. */
            bool includes(const StateSpacePtr &other) const;

            /** \brief Return true if \e other is a space that is either included (perhaps equal, perhaps a subspace)
                in this one, or all of its subspaces are included in this one. */
            bool covers(const StateSpacePtr &other) const;

            /** @} */

            /** @name Functionality specific to state spaces (to be implemented by derived state spaces)
                @{ */

            /** \brief Get the dimension of the space (not the dimension of the surrounding ambient space) */
            virtual unsigned int getDimension(void) const = 0;

            /** \brief Get the maximum value a call to distance() can return (or an upper bound).
                For unbounded state spaces, this function can return infinity.

                \note Tight upper bounds are preferred because the value of the extent is used in
                the automatic computation of parameters for planning. If the bounds are less tight,
                the automatically computed parameters will be less useful.*/
            virtual double getMaximumExtent(void) const = 0;

            /** \brief Bring the state within the bounds of the state space. For unbounded spaces this
                function can be a no-op. */
            virtual void enforceBounds(State *state) const = 0;

            /** \brief Check if a state is inside the bounding box. For unbounded spaces this function
                can always return true. */
            virtual bool satisfiesBounds(const State *state) const = 0;

            /** \brief Copy a state to another. The memory of source and destination should NOT overlap. */
            virtual void copyState(State *destination, const State *source) const = 0;

            /** \brief Computes distance to between two states. This function satisfies the properties of a
                metric and its return value will always be between 0 and getMaximumExtent() */
            virtual double distance(const State *state1, const State *state2) const = 0;

            /** \brief Many states contain a number of double values. This function provides a means to get the
                memory address of a double value from state \e state located at position \e index. The first double value
                is returned for \e index = 0. If \e index is too large (does not point to any double values in the state),
                the return value is NULL.

                \note This function does @b not map a state to an
                array of doubles. There may be components of a state
                that do not correspond to double values and they are
                'invisible' to this function. Furthermore, this
                function is slow and is not intended for use in the
                implementation of planners. */
            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function returns this length, for this state space, as a
                fraction of the space's maximum extent. */
            virtual double getLongestValidSegmentFraction(void) const;

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function sets this length as a fraction of the
                space's maximum extent.

                \note This function's effect is not considered until
                after setup() has been called. For immediate effects
                (i.e., during planning) use
                setValidSegmentCountFactor() */
            virtual void setLongestValidSegmentFraction(double segmentFraction);

            /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2 */
            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;

            /** \brief Set \e factor to be the value to multiply the
                return value of validSegmentCount(). By default, this
                value is 1. The higher the value, the smaller the size
                of the segments considered valid. The effect of this
                function is immediate (setup() does not need to be
                called). */
            void setValidSegmentCountFactor(unsigned int factor);

            /** \brief Get the value used to multiply the return value of validSegmentCount().*/
            unsigned int getValidSegmentCountFactor(void) const;

            /** \brief Checks whether two states are equal */
            virtual bool equalStates(const State *state1, const State *state2) const = 0;

            /** \brief Computes the state that lies at time @e t in [0, 1] on the segment that connects @e from state to @e to state.
                The memory location of @e state is not required to be different from the memory of either
                @e from or @e to. */
            virtual void interpolate(const State *from, const State *to, const double t, State *state) const = 0;

            /** \brief Allocate an instance of the default uniform state sampler for this space */
            virtual StateSamplerPtr allocDefaultStateSampler(void) const = 0;

            /** \brief Allocate an instance of the uniform state sampler for this space. This sampler will be allocated with the
                sampler allocator that was previously specified by setSamplerAllocator() or, if no sampler allocator was specified,
                allocDefaultStateSampler() is called */
            StateSamplerPtr allocStateSampler(void) const;

            /** \brief Set the sampler allocator to use */
            void setStateSamplerAllocator(const StateSamplerAllocator &ssa);

            /** \brief Allocate a state that can store a point in the described space */
            virtual State* allocState(void) const = 0;

            /** \brief Free the memory of the allocated state */
            virtual void freeState(State *state) const = 0;

            /** @} */


            /** @name Management of projections from this state space to Euclidean spaces
                @{ */

            /** \brief Register a projection for this state space under a specified name */
            void registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection);

            /** \brief Register the default projection for this state space */
            void registerDefaultProjection(const ProjectionEvaluatorPtr &projection);

            /** \brief Register the projections for this state space. Usually, this is at least the default
                projection. These are implicit projections, set by the implementation of the state space. This is called by setup(). */
            virtual void registerProjections(void);

            /** \brief Get the projection registered under a specific name */
            ProjectionEvaluatorPtr getProjection(const std::string &name) const;

            /** \brief Get the default projection */
            ProjectionEvaluatorPtr getDefaultProjection(void) const;

            /** \brief Check if a projection with a specified name is available */
            bool hasProjection(const std::string &name) const;

            /** \brief Check if a default projection is available */
            bool hasDefaultProjection(void) const;

            /** \brief Get all the registered projections */
            const std::map<std::string, ProjectionEvaluatorPtr>& getRegisteredProjections(void) const;

            /** @} */

            /** @name Debugging tools
                @{ */

            /** \brief Print a state to a stream */
            virtual void printState(const State *state, std::ostream &out) const;

            /** \brief Print the settings for this state space to a stream */
            virtual void printSettings(std::ostream &out) const;

            /** \brief Print the list of registered projections. This function is also called by printSettings() */
            virtual void printProjections(std::ostream &out) const;

            /** \brief Perform sanity checks for this state space. Throws an exception if failures are found.
                \note This checks if distances are always positive, whether the integration works as expected. */
            virtual void sanityChecks(void) const;

            /** \brief Print a Graphviz digraph that represents the containment diagram for all the instantiated state spaces */
            static void Diagram(std::ostream &out);

            /** @} */

            /** \brief Perform final setup steps. This function is
                automatically called by the SpaceInformation. If any
                default projections are to be registered, this call
                will set them. It is safe to call this function
                multiple times. */
            virtual void setup(void);

        protected:

            /** \brief The name used for the default projection */
            static const std::string DEFAULT_PROJECTION_NAME;

            /** \brief A type assigned for this state space */
            int                                           type_;

            /** \brief An optional state sampler allocator */
            StateSamplerAllocator                         ssa_;

            /** \brief The extent of this space at the time setup() was called */
            double                                        maxExtent_;

            /** \brief The fraction of the longest valid segment */
            double                                        longestValidSegmentFraction_;

            /** \brief The longest valid segment at the time setup() was called */
            double                                        longestValidSegment_;

            /** \brief The factor to multiply the value returned by validSegmentCount() */
            unsigned int                                  longestValidSegmentCountFactor_;

            /** \brief List of available projections */
            std::map<std::string, ProjectionEvaluatorPtr> projections_;

            /** \brief Interface used for console output */
            msg::Interface                                msg_;

        private:

            /** \brief State space name */
            std::string                                   name_;
        };

        /** \brief A space to allow the composition of state spaces */
        class CompoundStateSpace : public StateSpace
        {
        public:

            /** \brief Define the type of state allocated by this state space */
            typedef CompoundState StateType;

            /** \brief Construct an empty compound state space */
            CompoundStateSpace(void);

            /** \brief Construct a compound state space from a list of subspaces (\e components) and their corresponding weights (\e weights) */
            CompoundStateSpace(const std::vector<StateSpacePtr> &components, const std::vector<double> &weights);

            virtual ~CompoundStateSpace(void)
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const unsigned int index) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<T*>(getSubSpace(index).get());
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const std::string &name) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<T*>(getSubSpace(name).get());
            }

            virtual bool isCompound(void) const;

            virtual bool isHybrid(void) const;

            /** @name Management of contained subspaces
                @{ */

            /** \brief Adds a new state space as part of the compound state space. For computing distances within the compound
                state space, the weight of the component also needs to be specified. */
            virtual void addSubSpace(const StateSpacePtr &component, double weight);

            /** \brief Get the number of state spaces that make up the compound state space */
            unsigned int getSubSpaceCount(void) const;

            /** \brief Get a specific subspace from the compound state space */
            const StateSpacePtr& getSubSpace(const unsigned int index) const;

            /** \brief Get a specific subspace from the compound state space */
            const StateSpacePtr& getSubSpace(const std::string& name) const;

            /** \brief Get the index of a specific subspace from the compound state space */
            unsigned int getSubSpaceIndex(const std::string& name) const;

            /** \brief Check if a specific subspace is contained in this state space */
            bool hasSubSpace(const std::string &name) const;

            /** \brief Get the weight of a subspace from the compound state space (used in distance computation) */
            double getSubSpaceWeight(const unsigned int index) const;

            /** \brief Get the weight of a subspace from the compound state space (used in distance computation) */
            double getSubSpaceWeight(const std::string &name) const;

            /** \brief Set the weight of a subspace in the compound state space (used in distance computation) */
            void setSubSpaceWeight(const unsigned int index, double weight);

            /** \brief Set the weight of a subspace in the compound state space (used in distance computation) */
            void setSubSpaceWeight(const std::string &name, double weight);

            /** \brief Get the list of components */
            const std::vector<StateSpacePtr>& getSubSpaces(void) const;

            /** \brief Get the list of component weights */
            const std::vector<double>& getSubSpaceWeights(void) const;

            /** \brief Return true if the state space is locked. A value
                of true means that no further spaces can be added
                as components. */
            bool isLocked(void) const;

            /** @} */

            /** @name Functionality specific to the state space
                @{ */

            virtual unsigned int getDimension(void) const;

            virtual double getMaximumExtent(void) const;

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual double distance(const State *state1, const State *state2) const;

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function sets this length as a fraction of the space's
                maximum extent. The call is passed to all contained subspaces */
            virtual void setLongestValidSegmentFraction(double segmentFraction);

            /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2.
                This is the max() of the counts returned by contained subspaces. */
            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler(void) const;

            virtual State* allocState(void) const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            /** @} */

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void setup(void);

            /** \brief Lock this state space. This means no further
                spaces can be added as components.  This function can
                be for instance called from the constructor of a
                state space that inherits from CompoundStateSpace to
                prevent the user to add further components. */
            void lock(void);

        protected:

            /** \brief Allocate the state components. Called by allocState(). Usually called by derived state spaces. */
            void allocStateComponents(CompoundState *state) const;

            /** \brief The state spaces that make up the compound state space */
            std::vector<StateSpacePtr>    components_;

            /** \brief The number of components */
            unsigned int                  componentCount_;

            /** \brief The weight assigned to each component of the state space when computing the compound distance */
            std::vector<double>           weights_;

            /** \brief Flag indicating whether adding further components is allowed or not */
            bool                          locked_;

        };

        /** \addtogroup stateAndSpaceOperators
         *  @{
         */

        /** \brief Construct a compound state space from two existing
            state spaces. The components of this compound space are \e
            a (or the components of \e a, if \e a is compound) and \e
            b (or the components of \e b, if \e b is compound).
            State spaces are identified by name. Duplicates are checked
            for and added only once. If the compound state space would
            end up containing solely one component, that component is returned
            instead. */
        StateSpacePtr operator+(const StateSpacePtr &a, const StateSpacePtr &b);

        /** \brief Construct a compound state space that contains
            subspaces only from \e a. If \e a is compound, \e b (or
            the components from \e b, if \e b is compound) are removed
            and the remaining components are returned as a compound
            state space. If the compound space would end up containing solely
            one component, that component is returned instead. */
        StateSpacePtr operator-(const StateSpacePtr &a, const StateSpacePtr &b);

        /** \brief Construct a compound state space that contains
            subspaces only from \e a, except for maybe the one named \e name */
        StateSpacePtr operator-(const StateSpacePtr &a, const std::string &name);

        /** \brief Construct a compound state space that contains
            subspaces that are in both \e a and \e b */
        StateSpacePtr operator*(const StateSpacePtr &a, const StateSpacePtr &b);
        /** @} */

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) on a
            component by component basis. State spaces are matched by
            name. If the state space \e destS contains any subspace
            whose name matches any subspace of the state space \e
            sourceS, the corresponding state components are
            copied. The return value is either 0 (no data copied), 1
            (some data copied), 2 (all data copied) */
        int copyStateData(const StateSpacePtr &destS, State *dest,
                          const StateSpacePtr &sourceS, const State *source);
    }
}

#endif
