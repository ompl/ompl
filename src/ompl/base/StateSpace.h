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
#include "ompl/base/GenericParam.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateSpace */
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /** \class ompl::base::StateSpacePtr
            \brief A shared pointer wrapper for ompl::base::StateSpace */

        /** \brief Representation of a space in which planning can be
            performed. Topology specific sampling, interpolation and distance
            are defined.

            See \ref implementingStateSpaces. */
        class StateSpace
        {
        public:
            // non-copyable
            StateSpace(const StateSpace &) = delete;
            StateSpace &operator=(const StateSpace &) = delete;

            /** \brief Define the type of state allocated by this space */
            using StateType = ompl::base::State;

            /** \brief Constructor. Assigns a @b unique name to the space */
            StateSpace();

            virtual ~StateSpace();

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, StateSpace *>));

                return static_cast<T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, StateSpace *>));

                return static_cast<const T *>(this);
            }

            /** \brief Representation of the address of a substate in a state. This structure stores the indexing
             * information needed to access a particular substate of a state */
            struct SubstateLocation
            {
                /** \brief In a complex state space there may be multiple
                    compound state spaces that make up an even larger
                    compound space.  This array indicates the sequence of
                    indices of the subspaces that need to be followed to
                    get to the component of the state that is of interest. */
                std::vector<std::size_t> chain;

                /** \brief The space that is reached if the chain above is followed on the state space */
                const StateSpace *space;
            };

            /** \brief Representation of the address of a value in a state. This structure stores the indexing
             * information needed to access elements of a state (no pointer values are stored) */
            struct ValueLocation
            {
                /** \brief Location of the substate that contains the pointed to value */
                SubstateLocation stateLocation;

                /** \brief The index of the value to be accessed, within the substate location above */
                std::size_t index;
            };

            /** \brief Flags to use in a bit mask for state space sanity checks. Some basic checks do not have flags
               associated (they are always executed; for example,
                whether copyState() works as expected) */
            enum SanityChecks
            {

                /// \brief Check whether the distances between non-equal states is strictly positive
                /// (StateSpace::distance())
                STATESPACE_DISTANCE_DIFFERENT_STATES = (1 << 1),

                /// \brief Check whether the distance function is symmetric (StateSpace::distance())
                STATESPACE_DISTANCE_SYMMETRIC = (1 << 2),

                /// \brief Check whether calling StateSpace::interpolate() works as expected
                STATESPACE_INTERPOLATION = (1 << 3),

                /// \brief Check whether the triangle inequality holds when using StateSpace::interpolate() and
                /// StateSpace::distance()
                STATESPACE_TRIANGLE_INEQUALITY = (1 << 4),

                /// \brief Check whether the StateSpace::distance() is bounded by StateSpace::getExtent()
                STATESPACE_DISTANCE_BOUND = (1 << 5),

                /// \brief Check whether sampled states are always within bounds
                STATESPACE_RESPECT_BOUNDS = (1 << 6),

                /// \brief Check that enforceBounds() does not modify the contents of states that are within bounds
                STATESPACE_ENFORCE_BOUNDS_NO_OP = (1 << 7),

                /// \brief Check whether the StateSpace::serialize() and StateSpace::deserialize() work as expected
                STATESPACE_SERIALIZATION = (1 << 8)
            };

            /** @name Generic functionality for state spaces
                @{ */

            /** \brief Check if the state space is compound */
            virtual bool isCompound() const;

            /** \brief Check if the set of states is discrete

                \note In fact, because of limited numerical precision,
                the representation of all spaces is discrete; this
                function returns true if the corresponding
                mathematical object is a discrete one. */
            virtual bool isDiscrete() const;

            /** \brief Check if this is a hybrid state space (i.e., both discrete and continuous components exist)*/
            virtual bool isHybrid() const;

            /** \brief Return true if the distance function associated with the space
                is a metric */
            virtual bool isMetricSpace() const
            {
                return true;
            }

            /** \brief Check if the distance function on this state space is symmetric, i.e. distance(s1,s2) =
             * distance(s2,s1). Default implementation returns true.*/
            virtual bool hasSymmetricDistance() const;

            /** \brief Check if the interpolation function on this state space is symmetric, i.e. interpolate(from, to,
             * t, state) = interpolate(to, from, 1-t, state). Default implementation returns true.*/
            virtual bool hasSymmetricInterpolate() const;

            /** \brief Get the name of the state space */
            const std::string &getName() const;

            /** \brief Set the name of the state space */
            void setName(const std::string &name);

            /** \brief Get the type of the state space. The type can be
                used to verify whether two space instances are of
                the same type (e.g., SO2) */
            int getType() const
            {
                return type_;
            }

            /** \brief Return true if \e other is a space included (perhaps equal, perhaps a subspace) in this one. */
            bool includes(const StateSpacePtr &other) const;

            /** \brief Return true if \e other is a space included (perhaps equal, perhaps a subspace) in this one. */
            bool includes(const StateSpace *other) const;

            /** \brief Return true if \e other is a space that is either included (perhaps equal, perhaps a subspace)
                in this one, or all of its subspaces are included in this one. */
            bool covers(const StateSpacePtr &other) const;

            /** \brief Return true if \e other is a space that is either included (perhaps equal, perhaps a subspace)
                in this one, or all of its subspaces are included in this one. */
            bool covers(const StateSpace *other) const;

            /** \brief Get the parameters for this space */
            ParamSet &params()
            {
                return params_;
            }

            /** \brief Get the parameters for this space */
            const ParamSet &params() const
            {
                return params_;
            }

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function returns this length, for this state space, as a
                fraction of the space's maximum extent. */
            virtual double getLongestValidSegmentFraction() const;

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

            /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e
             * state2 */
            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;

            /** \brief Set \e factor to be the value to multiply the
                return value of validSegmentCount(). By default, this
                value is 1. The higher the value, the smaller the size
                of the segments considered valid. The effect of this
                function is immediate (setup() does not need to be
                called). */
            virtual void setValidSegmentCountFactor(unsigned int factor);

            /** \brief Get the value used to multiply the return value of validSegmentCount().*/
            virtual unsigned int getValidSegmentCountFactor() const;

            /** \brief Get the longest valid segment at the time setup() was called. */
            virtual double getLongestValidSegmentLength() const;

            /** \brief Compute an array of ints that uniquely identifies the structure of the state space.
                The first element of the signature is the number of integers that follow */
            virtual void computeSignature(std::vector<int> &signature) const;

            /** @} */

            /** @name Functionality specific to state spaces (to be implemented by derived state spaces)
                @{ */

            /** \brief Get the dimension of the space (not the dimension of the surrounding ambient space) */
            virtual unsigned int getDimension() const = 0;

            /** \brief Get the maximum value a call to distance() can return (or an upper bound).
                For unbounded state spaces, this function can return infinity.

                \note Tight upper bounds are preferred because the value of the extent is used in
                the automatic computation of parameters for planning. If the bounds are less tight,
                the automatically computed parameters will be less useful.*/
            virtual double getMaximumExtent() const = 0;

            /** \brief Get a measure of the space (this can be thought of as a generalization of volume) */
            virtual double getMeasure() const = 0;

            /** \brief Bring the state within the bounds of the state space. For unbounded spaces this
                function can be a no-op. */
            virtual void enforceBounds(State *state) const = 0;

            /** \brief Check if a state is inside the bounding box. For unbounded spaces this function
                can always return true. */
            virtual bool satisfiesBounds(const State *state) const = 0;

            /** \brief Copy a state to another. The memory of source and destination should NOT overlap.
                \note For more advanced state copying methods (partial copy, for example), see \ref advancedStateCopy.
               */
            virtual void copyState(State *destination, const State *source) const = 0;

            /** \brief Clone a state*/
            State *cloneState(const State *source) const;

            /** \brief Computes distance between two states. This function satisfies the properties of a
                metric if isMetricSpace() is true, and its return value will always be between 0 and getMaximumExtent()
               */
            virtual double distance(const State *state1, const State *state2) const = 0;

            /** \brief Get the number of chars in the serialization of a state in this space */
            virtual unsigned int getSerializationLength() const;

            /** \brief Write the binary representation of \e state to \e serialization */
            virtual void serialize(void *serialization, const State *state) const;

            /** \brief Read the binary representation of a state from \e serialization and write it to \e state */
            virtual void deserialize(State *state, const void *serialization) const;

            /** \brief Checks whether two states are equal */
            virtual bool equalStates(const State *state1, const State *state2) const = 0;

            /** \brief Computes the state that lies at time @e t in [0, 1] on the segment that connects @e from state to
               @e to state.
                The memory location of @e state is not required to be different from the memory of either
                @e from or @e to. */
            virtual void interpolate(const State *from, const State *to, double t, State *state) const = 0;

            /** \brief Allocate an instance of the default uniform state sampler for this space */
            virtual StateSamplerPtr allocDefaultStateSampler() const = 0;

            /** \brief Allocate an instance of the state sampler for this space. This sampler will be allocated with the
                sampler allocator that was previously specified by setStateSamplerAllocator() or, if no sampler
               allocator was specified,
                allocDefaultStateSampler() is called */
            virtual StateSamplerPtr allocStateSampler() const;

            /** \brief Set the sampler allocator to use */
            void setStateSamplerAllocator(const StateSamplerAllocator &ssa);

            /** \brief Clear the state sampler allocator (reset to default) */
            void clearStateSamplerAllocator();

            /** \brief Allocate a state that can store a point in the described space */
            virtual State *allocState() const = 0;

            /** \brief Free the memory of the allocated state */
            virtual void freeState(State *state) const = 0;

            /** @} */

            /** @name Functionality specific to accessing real values in a state
                @{ */

            /** \brief Many states contain a number of double values. This function provides a means to get the
                memory address of a double value from state \e state located at position \e index. The first double
               value
                is returned for \e index = 0. If \e index is too large (does not point to any double values in the
               state),
                the return value is nullptr.

                \note This function does @b not map a state to an
                array of doubles. There may be components of a state
                that do not correspond to double values and they are
                'invisible' to this function. Furthermore, this
                function is @b slow and is not intended for use in the
                implementation of planners. Ideally, state values should not be accessed by index. If accessing of
               individual state elements
                is however needed, getValueAddressAtLocation() provides a faster implementation. */
            virtual double *getValueAddressAtIndex(State *state, unsigned int index) const;

            /** \brief Const variant of the same function as above; */
            const double *getValueAddressAtIndex(const State *state, unsigned int index) const;

            /** \brief Get the locations of values of type double contained in a state from this space. The order of the
               values is
                consistent with getValueAddressAtIndex(). The setup() function must have been previously called. */
            const std::vector<ValueLocation> &getValueLocations() const;

            /** \brief Get the named locations of values of type double contained in a state from this space.
                The setup() function must have been previously called. */
            const std::map<std::string, ValueLocation> &getValueLocationsByName() const;

            /** \brief Get a pointer to the double value in \e state that \e loc points to */
            double *getValueAddressAtLocation(State *state, const ValueLocation &loc) const;

            /** \brief Const variant of the same function as above; */
            const double *getValueAddressAtLocation(const State *state, const ValueLocation &loc) const;

            /** \brief Get a pointer to the double value in \e state that \e name points to */
            double *getValueAddressAtName(State *state, const std::string &name) const;

            /** \brief Const variant of the same function as above; */
            const double *getValueAddressAtName(const State *state, const std::string &name) const;

            /** \brief Copy all the real values from a state \e source to the array \e reals using
             * getValueAddressAtLocation() */
            virtual void copyToReals(std::vector<double> &reals, const State *source) const;

            /** \brief Copy the values from \e reals to the state \e destination using getValueAddressAtLocation() */
            virtual void copyFromReals(State *destination, const std::vector<double> &reals) const;

            /** @} */

            /** @name Management of projections from this state space to Euclidean spaces
                @{ */

            /** \brief Register a projection for this state space under a specified name */
            void registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection);

            /** \brief Register the default projection for this state space */
            void registerDefaultProjection(const ProjectionEvaluatorPtr &projection);

            /** \brief Register the projections for this state space. Usually, this is at least the default
                projection. These are implicit projections, set by the implementation of the state space. This is called
               by setup(). */
            virtual void registerProjections();

            /** \brief Get the projection registered under a specific name */
            ProjectionEvaluatorPtr getProjection(const std::string &name) const;

            /** \brief Get the default projection */
            ProjectionEvaluatorPtr getDefaultProjection() const;

            /** \brief Check if a projection with a specified name is available */
            bool hasProjection(const std::string &name) const;

            /** \brief Check if a default projection is available */
            bool hasDefaultProjection() const;

            /** \brief Get all the registered projections */
            const std::map<std::string, ProjectionEvaluatorPtr> &getRegisteredProjections() const;

            /** @} */

            /** @name Debugging tools
                @{ */

            /** \brief Print a state to a stream */
            virtual void printState(const State *state, std::ostream &out = std::cout) const;

            /** \brief Print the settings for this state space to a stream */
            virtual void printSettings(std::ostream &out) const;

            /** \brief Print the list of registered projections. This function is also called by printSettings() */
            virtual void printProjections(std::ostream &out) const;

            /** \brief Perform sanity checks for this state space. Throws an exception if failures are found.
                \note This checks if distances are always positive, whether the integration works as expected, etc. */
            virtual void sanityChecks(double zero, double eps, unsigned int flags) const;

            /** \brief Convenience function that allows derived state spaces to choose which checks
                should pass (see SanityChecks flags) and how strict the checks are. This just calls sanityChecks() with
               some default arguments. */
            virtual void sanityChecks() const;

            /** \brief Print a Graphviz digraph that represents the containment diagram for the state space */
            void diagram(std::ostream &out) const;

            /** \brief Print the list of all contained state space instances */
            void list(std::ostream &out) const;

            /** \brief Print a Graphviz digraph that represents the containment diagram for all the instantiated state
             * spaces */
            static void Diagram(std::ostream &out);

            /** \brief Print the list of available state space instances */
            static void List(std::ostream &out);

            /** @} */

            /** @name Operations with substates
                @{ */

            /** \brief Allocate a sampler that actually samples only components that are part of \e subspace */
            StateSamplerPtr allocSubspaceStateSampler(const StateSpacePtr &subspace) const;

            /** \brief Allocate a sampler that actually samples only components that are part of \e subspace */
            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const;

            /** \brief Get the substate of \e state that is pointed to by \e loc */
            State *getSubstateAtLocation(State *state, const SubstateLocation &loc) const;

            /** \brief Get the substate of \e state that is pointed to by \e loc */
            const State *getSubstateAtLocation(const State *state, const SubstateLocation &loc) const;

            /** \brief Get the list of known substate locations (keys of the map corrspond to names of subspaces) */
            const std::map<std::string, SubstateLocation> &getSubstateLocationsByName() const;

            /** \brief Get the set of subspaces that this space and \e other have in common. The computed list of \e
               subspaces does
                not contain spaces that cover each other, even though they may be common, as that is redundant
               information. */
            void getCommonSubspaces(const StateSpacePtr &other, std::vector<std::string> &subspaces) const;

            /** \brief Get the set of subspaces that this space and \e other have in common. The computed list of \e
               subspaces does
                not contain spaces that cover each other, even though they may be common, as that is redundant
               information. */
            void getCommonSubspaces(const StateSpace *other, std::vector<std::string> &subspaces) const;

            /** \brief Compute the location information for various components of the state space. Either this function
               or setup() must be
                called before any calls to getValueAddressAtName(), getValueAddressAtLocation() (and other functions
               where those are used). */
            virtual void computeLocations();

            /** @} */

            /** \brief Perform final setup steps. This function is
                automatically called by the SpaceInformation. If any
                default projections are to be registered, this call
                will set them and call their setup() functions. It is
                safe to call this function multiple times. At a
                subsequent call, projections that have been previously
                user configured are not re-instantiated, but their
                setup() method is still called. */
            virtual void setup();

        protected:
            /** \brief The name used for the default projection */
            static const std::string DEFAULT_PROJECTION_NAME;

            /** \brief A type assigned for this state space */
            int type_;

            /** \brief An optional state sampler allocator */
            StateSamplerAllocator ssa_;

            /** \brief The extent of this space at the time setup() was called */
            double maxExtent_;

            /** \brief The fraction of the longest valid segment */
            double longestValidSegmentFraction_;

            /** \brief The longest valid segment at the time setup() was called */
            double longestValidSegment_;

            /** \brief The factor to multiply the value returned by validSegmentCount(). Rarely used but useful for
             * things like doubling the resolution */
            unsigned int longestValidSegmentCountFactor_;

            /** \brief List of available projections */
            std::map<std::string, ProjectionEvaluatorPtr> projections_;

            /** \brief The set of parameters for this space */
            ParamSet params_;

            /** \brief The value locations for all varliables of type double contained in a state;
                The locations point to values in the same order as that returned by getValueAddressAtIndex() */
            std::vector<ValueLocation> valueLocationsInOrder_;

            /** \brief All the known value locations, by name. The names of state spaces access the first element of a
               state.
                RealVectorStateSpace dimensions are used to access individual dimensions. */
            std::map<std::string, ValueLocation> valueLocationsByName_;

            /** \brief All the known substat locations, by name. */
            std::map<std::string, SubstateLocation> substateLocationsByName_;

        private:
            /** \brief State space name */
            std::string name_;
        };

        /** \brief A space to allow the composition of state spaces */
        class CompoundStateSpace : public StateSpace
        {
        public:
            /** \brief Define the type of state allocated by this state space */
            using StateType = ompl::base::CompoundState;

            /** \brief Construct an empty compound state space */
            CompoundStateSpace();

            /** \brief Construct a compound state space from a list of subspaces (\e components) and their corresponding
             * weights (\e weights) */
            CompoundStateSpace(const std::vector<StateSpacePtr> &components, const std::vector<double> &weights);

            ~CompoundStateSpace() override = default;

            /** \brief Cast a component of this instance to a desired type. */
            template <class T>
            T *as(const unsigned int index) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, StateSpace *>));

                return static_cast<T *>(getSubspace(index).get());
            }

            /** \brief Cast a component of this instance to a desired type. */
            template <class T>
            T *as(const std::string &name) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, StateSpace *>));

                return static_cast<T *>(getSubspace(name).get());
            }

            bool isCompound() const override;

            bool isHybrid() const override;

            /** @name Management of contained subspaces
                @{ */

            /** \brief Adds a new state space as part of the compound state space. For computing distances within the
               compound
                state space, the weight of the component also needs to be specified. */
            void addSubspace(const StateSpacePtr &component, double weight);

            /** \brief Get the number of state spaces that make up the compound state space */
            unsigned int getSubspaceCount() const;

            /** \brief Get a specific subspace from the compound state space */
            const StateSpacePtr &getSubspace(unsigned int index) const;

            /** \brief Get a specific subspace from the compound state space */
            const StateSpacePtr &getSubspace(const std::string &name) const;

            /** \brief Get the index of a specific subspace from the compound state space */
            unsigned int getSubspaceIndex(const std::string &name) const;

            /** \brief Check if a specific subspace is contained in this state space */
            bool hasSubspace(const std::string &name) const;

            /** \brief Get the weight of a subspace from the compound state space (used in distance computation) */
            double getSubspaceWeight(unsigned int index) const;

            /** \brief Get the weight of a subspace from the compound state space (used in distance computation) */
            double getSubspaceWeight(const std::string &name) const;

            /** \brief Set the weight of a subspace in the compound state space (used in distance computation) */
            void setSubspaceWeight(unsigned int index, double weight);

            /** \brief Set the weight of a subspace in the compound state space (used in distance computation) */
            void setSubspaceWeight(const std::string &name, double weight);

            /** \brief Get the list of components */
            const std::vector<StateSpacePtr> &getSubspaces() const;

            /** \brief Get the list of component weights */
            const std::vector<double> &getSubspaceWeights() const;

            /** \brief Return true if the state space is locked. A value
                of true means that no further spaces can be added
                as components. */
            bool isLocked() const;

            /** \brief Lock this state space. This means no further
                spaces can be added as components.  This function can
                be for instance called from the constructor of a
                state space that inherits from CompoundStateSpace to
                prevent the user to add further components. */
            void lock();
            /** @} */

            /** @name Operations with substates
                @{ */

            StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const override;

            /** @} */

            /** @name Functionality specific to the state space
                @{ */

            unsigned int getDimension() const override;

            double getMaximumExtent() const override;

            double getMeasure() const override;

            void enforceBounds(State *state) const override;

            bool satisfiesBounds(const State *state) const override;

            void copyState(State *destination, const State *source) const override;

            unsigned int getSerializationLength() const override;

            void serialize(void *serialization, const State *state) const override;

            void deserialize(State *state, const void *serialization) const override;

            double distance(const State *state1, const State *state2) const override;

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function sets this length as a fraction of the space's
                maximum extent. The call is passed to all contained subspaces */
            void setLongestValidSegmentFraction(double segmentFraction) override;

            /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e
               state2.
                This is the max() of the counts returned by contained subspaces. */
            unsigned int validSegmentCount(const State *state1, const State *state2) const override;

            bool equalStates(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;

            StateSamplerPtr allocDefaultStateSampler() const override;

            State *allocState() const override;

            void freeState(State *state) const override;

            double *getValueAddressAtIndex(State *state, unsigned int index) const override;

            /** @} */

            void printState(const State *state, std::ostream &out) const override;

            void printSettings(std::ostream &out) const override;

            void computeLocations() override;

            void setup() override;

        protected:
            /** \brief Allocate the state components. Called by allocState(). Usually called by derived state spaces. */
            void allocStateComponents(CompoundState *state) const;

            /** \brief The state spaces that make up the compound state space */
            std::vector<StateSpacePtr> components_;

            /** \brief The number of components */
            unsigned int componentCount_{0u};

            /** \brief The weight assigned to each component of the state space when computing the compound distance */
            std::vector<double> weights_;

            /** \brief The sum of all the weights in \e weights_ */
            double weightSum_{0.0};

            /** \brief Flag indicating whether adding further components is allowed or not */
            bool locked_{false};
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

        /** \defgroup advancedStateCopy Advanced methods for copying states
         *  @{
         */

        /** \brief The possible outputs for an advanced copy operation */
        enum AdvancedStateCopyOperation
        {
            /** \brief No data was copied */
            NO_DATA_COPIED = 0,

            /** \brief Some data was copied */
            SOME_DATA_COPIED = 1,

            /** \brief All data was copied */
            ALL_DATA_COPIED = 2
        };

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) on a
            component by component basis. State spaces are matched by
            name. If the state space \e destS contains any subspace
            whose name matches any subspace of the state space \e
            sourceS, the corresponding state components are
            copied. */
        AdvancedStateCopyOperation copyStateData(const StateSpacePtr &destS, State *dest, const StateSpacePtr &sourceS,
                                                 const State *source);

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) on a
            component by component basis. State spaces are matched by
            name. If the state space \e destS contains any subspace
            whose name matches any subspace of the state space \e
            sourceS, the corresponding state components are
            copied. */
        AdvancedStateCopyOperation copyStateData(const StateSpace *destS, State *dest, const StateSpace *sourceS,
                                                 const State *source);

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) but only
            for the subspaces indicated by name in \e subspaces. This
            uses StateSpace::getSubstateLocationsByName().
            \note For efficiency reasons it is a good idea usually to make sure the elements of \e subspaces are not
           subspaces of each other */
        AdvancedStateCopyOperation copyStateData(const StateSpacePtr &destS, State *dest, const StateSpacePtr &sourceS,
                                                 const State *source, const std::vector<std::string> &subspaces);

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) but only
            for the subspaces indicated by name in \e subspaces. This
            uses StateSpace::getSubstateLocationsByName().
            \note For efficiency reasons it is a good idea usually to make sure the elements of \e subspaces are not
           subspaces of each other */
        AdvancedStateCopyOperation copyStateData(const StateSpace *destS, State *dest, const StateSpace *sourceS,
                                                 const State *source, const std::vector<std::string> &subspaces);
        /** @} */
    }
}

#endif
