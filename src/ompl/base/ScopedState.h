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

#ifndef OMPL_BASE_SCOPED_STATE_
#define OMPL_BASE_SCOPED_STATE_

#include "ompl/base/SpaceInformation.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include <utility>

namespace ompl
{
    namespace base
    {
        /** \brief Definition of a scoped state.

            This class allocates a state of a desired type using the
            allocation mechanism of the corresponding state space.
            The state is then freed when the instance goes out of
            scope using the corresponding free mechanism. */
        template <class T = StateSpace>
        class ScopedState
        {
            /** \brief Make sure the type we are allocating is indeed from a state space */
            BOOST_CONCEPT_ASSERT((boost::Convertible<T *, StateSpace *>));

            /** \brief Make sure the type we are allocating is indeed a state */
            BOOST_CONCEPT_ASSERT((boost::Convertible<typename T::StateType *, State *>));

        public:
            /** \brief The type of the contained state */
            using StateType = typename T::StateType;

            /** \brief Given the space that we are working with,
                allocate a state from the corresponding
                state space.  */
            explicit ScopedState(const SpaceInformationPtr &si) : space_(si->getStateSpace())
            {
                State *s = space_->allocState();

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType *>(s);
            }

            /** \brief Given the state space that we are working with,
                allocate a state. */
            explicit ScopedState(StateSpacePtr space) : space_(std::move(space))
            {
                State *s = space_->allocState();

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType *>(s);
            }

            /** \brief Copy constructor */
            ScopedState(const ScopedState<T> &other) : space_(other.getSpace())
            {
                State *s = space_->allocState();
                state_ = static_cast<StateType *>(s);
                space_->copyState(s, static_cast<const State *>(other.get()));
            }

            /** \brief Copy constructor that allows instantiation from states of other type */
            template <class O>
            ScopedState(const ScopedState<O> &other) : space_(other.getSpace())
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O *, StateSpace *>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType *, State *>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                State *s = space_->allocState();
                state_ = static_cast<StateType *>(s);
                space_->copyState(s, static_cast<const State *>(other.get()));
            }

            /** \brief Given the state space that we are working with,
                allocate a state and fill that state with a given value. */
            ScopedState(StateSpacePtr space, const State *state) : space_(std::move(space))
            {
                State *s = space_->allocState();
                space_->copyState(s, state);

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType *>(s);
            }

            /** \brief Free the memory of the internally allocated state */
            ~ScopedState()
            {
                space_->freeState(state_);
            }

            /** \brief Get the state space that the state corresponds to */
            const StateSpacePtr &getSpace() const
            {
                return space_;
            }

            /** \brief Assignment operator */
            ScopedState<T> &operator=(const ScopedState<T> &other)
            {
                if (&other != this)
                {
                    space_->freeState(state_);
                    space_ = other.getSpace();

                    State *s = space_->allocState();
                    state_ = static_cast<StateType *>(s);
                    space_->copyState(s, static_cast<const State *>(other.get()));
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T> &operator=(const State *other)
            {
                if (other != static_cast<State *>(state_))
                {
                    // ideally, we should use a dynamic_cast and throw an
                    // exception in case other does not cast to
                    // const StateType*. However, RTTI may not be
                    // available across shared library boundaries, so we
                    // do not use it

                    space_->copyState(static_cast<State *>(state_), other);
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T> &operator=(const State &other)
            {
                if (&other != static_cast<State *>(state_))
                {
                    // ideally, we should use a dynamic_cast and throw an
                    // exception in case &other does not cast to
                    // const StateType*. However, RTTI may not be
                    // available across shared library boundaries, so we
                    // do not use it

                    space_->copyState(static_cast<State *>(state_), &other);
                }
                return *this;
            }

            /** \brief Assignment operator that allows conversion of states */
            template <class O>
            ScopedState<T> &operator=(const ScopedState<O> &other)
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O *, StateSpace *>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType *, State *>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                if (reinterpret_cast<const void *>(&other) != reinterpret_cast<const void *>(this))
                {
                    space_->freeState(state_);
                    space_ = other.getSpace();

                    State *s = space_->allocState();
                    state_ = static_cast<StateType *>(s);
                    space_->copyState(s, static_cast<const State *>(other.get()));
                }
                return *this;
            }

            /** \brief Partial assignment operator. Only sets the double values of the state to specified real values */
            ScopedState<T> &operator=(const std::vector<double> &reals)
            {
                for (unsigned int i = 0; i < reals.size(); ++i)
                    if (double *va = space_->getValueAddressAtIndex(state_, i))
                        *va = reals[i];
                    else
                        break;
                return *this;
            }

            /** \brief Partial assignment operator. Only sets the double values of the state to a fixed value */
            ScopedState<T> &operator=(const double value)
            {
                unsigned int index = 0;
                while (double *va = space_->getValueAddressAtIndex(state_, index++))
                    *va = value;
                return *this;
            }

            /** \brief Checks equality of two states */
            template <class O>
            bool operator==(const ScopedState<O> &other) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O *, StateSpace *>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType *, State *>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                return space_->equalStates(static_cast<const State *>(state_), static_cast<const State *>(other.get()));
            }

            /** \brief Checks equality of two states */
            template <class O>
            bool operator!=(const ScopedState<O> &other) const
            {
                return !(*this == other);
            }

            /** \brief Extract a state that corresponds to the
                components in state space \e s. Those components will
                have the same value as the current state (only the
                ones included in the current state; others will be
                uninitialised). Note: a new state is constructed and data is copied. */
            const ScopedState<> operator[](const StateSpacePtr &s) const;

            /** \brief Access the \e index<sup>th</sup> double value this state contains. */
            double &operator[](const unsigned int index)
            {
                double *val = space_->getValueAddressAtIndex(state_, index);
                if (val == nullptr)
                    throw Exception("Index out of bounds");
                return *val;
            }

            /** \brief Access the \e index<sup>th</sup> double value this state contains. */
            double operator[](const unsigned int index) const
            {
                const double *val = space_->getValueAddressAtIndex(state_, index);
                if (val == nullptr)
                    throw Exception("Index out of bounds");
                return *val;
            }

            /** \brief Access a double value from this state contains using its name. */
            double &operator[](const std::string &name)
            {
                const std::map<std::string, StateSpace::ValueLocation> &vm = space_->getValueLocationsByName();
                auto it = vm.find(name);
                if (it != vm.end())
                {
                    double *val = space_->getValueAddressAtLocation(state_, it->second);
                    if (val != nullptr)
                        return *val;
                }
                throw Exception("Name '" + name + "' not known");
            }

            /** \brief Access a double value from this state contains using its name. */
            double operator[](const std::string &name) const
            {
                const std::map<std::string, StateSpace::ValueLocation> &vm = space_->getValueLocationsByName();
                auto it = vm.find(name);
                if (it != vm.end())
                {
                    const double *val = space_->getValueAddressAtLocation(state_, it->second);
                    if (val != nullptr)
                        return *val;
                }
                throw Exception("Name '" + name + "' not known");
            }

            /** \brief Compute the distance to another state. */
            template <class O>
            double distance(const ScopedState<O> &other) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O *, StateSpace *>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType *, State *>));
                return distance(other.get());
            }

            /** \brief Compute the distance to another state. */
            double distance(const State *state) const
            {
                return space_->distance(static_cast<const State *>(state_), state);
            }

            /** \brief Set this state to a random value (uniform) */
            void random()
            {
                if (!sampler_)
                    sampler_ = space_->allocStateSampler();
                sampler_->sampleUniform(state_);
            }

            /** \brief Enforce the bounds on the maintained state */
            void enforceBounds()
            {
                space_->enforceBounds(state_);
            }

            /** \brief Check if the maintained state satisfies bounds */
            bool satisfiesBounds() const
            {
                return space_->satisfiesBounds(state_);
            }

            /** \brief Return the real values corresponding to this
                state. If a conversion is not possible, an exception
                is thrown.*/
            std::vector<double> reals() const
            {
                std::vector<double> r;
                unsigned int index = 0;
                while (double *va = space_->getValueAddressAtIndex(state_, index++))
                    r.push_back(*va);
                return r;
            }

            /** \brief Print this state to a stream */
            void print(std::ostream &out = std::cout) const
            {
                space_->printState(state_, out);
            }

            /** \brief De-references to the contained state */
            StateType &operator*()
            {
                return *state_;
            }

            /** \brief De-references to the contained state */
            const StateType &operator*() const
            {
                return *state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType *operator->()
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            const StateType *operator->() const
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType *get()
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            const StateType *get() const
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state (used for Python bindings) */
            StateType *operator()() const
            {
                return state_;
            }

        private:
            StateSpacePtr space_;
            StateSamplerPtr sampler_;
            StateType *state_;
        };

        /** \addtogroup stateAndSpaceOperators Operators for States and State Spaces

           These operators are intended to simplify code that
           manipulates states and state spaces. They rely on the fact
           that state spaces have unique names. Here are some examples
           for using these operators:
          \code{.cpp}
           // Assume X, Y, Z, W are state space instances, none of
           // which inherits from ompl::base::CompoundStateSpace.
           // Denote a compound state space as C[...], where "..." is the
           // list of subspaces.

           ompl::base::StateSpacePtr X;
           ompl::base::StateSpacePtr Y;
           ompl::base::StateSpacePtr Z;
           ompl::base::StateSpacePtr W;

           // the following line will construct a state space C1 = C[X, Y]
           ompl::base::StateSpacePtr C1 = X + Y;

           // the following line will construct a state space C2 = C[X, Y, Z]
           ompl::base::StateSpacePtr C2 = C1 + Z;

           // the following line will leave C2 as C[X, Y, Z]
           ompl::base::StateSpacePtr C2 = C1 + C2;

           // the following line will construct a state space C2 = C[X, Y, Z, W]
           ompl::base::StateSpacePtr C2 = C2 + W;

           // the following line will construct a state space C3 = C[X, Z, Y]
           ompl::base::StateSpacePtr C3 = X + Z + Y;

           // the following line will construct a state space C4 = C[Z, W]
           ompl::base::StateSpacePtr C4 = C2 - C1;

           // the following line will construct a state space C5 = W
           ompl::base::StateSpacePtr C5 = C2 - C3;

           // the following line will construct an empty state space C6 = C[]
           ompl::base::StateSpacePtr C6 = X - X;

           // the following line will construct a state space C7 = Y
           ompl::base::StateSpacePtr C7 = Y + C6;
          \endcode
           These state spaces can be used when operating with states:
          \code{.cpp}
           ompl::base::ScopedState<> sX(X);
           ompl::base::ScopedState<> sXY(X + Y);
           ompl::base::ScopedState<> sY(Y);
           ompl::base::ScopedState<> sZX(Z + X);
           ompl::base::ScopedState<> sXZW(X + Z + W);

           // the following line will copy the content of the state sX to
           // the corresponding locations in sXZW. The components of the state
           // corresponding to the Z and W state spaces are not touched
           sX >> sXZW;

           // the following line will initialize the X component of sXY with
           // the X component of sXZW;
           sXY << sXZW;

           // the following line will initialize both components of sZX, using
           // the X and Z components of sXZW;
           sZX << sXZW;

           // the following line compares the concatenation of states sX and sY with sXY
           // the concatenation will automatically construct the state space X + Y and a state
           // from that state space containing the information from sX and sY. Since sXY is
           // constructed from the state space X + Y, the two are comparable.
           bool eq = (sX ^ sY) == sXY;
          \endcode
            @{
         */

        /** \brief Overload stream output operator. Calls ompl::base::StateSpace::printState() */
        template <class T>
        inline std::ostream &operator<<(std::ostream &out, const ScopedState<T> &state)
        {
            state.print(out);
            return out;
        }

        /** \brief This is a fancy version of the assignment
            operator. It is a partial assignment, in some sense. The
            difference is that if the states are part of compound
            state spaces, the data is copied from \e from to \e to on a
            component by component basis. State spaces are matched by
            name. If the state space for \e to contains any subspace
            whose name matches any subspace of the state space for \e
            from, the corresponding state components are copied. */
        template <class T, class Y>
        inline ScopedState<T> &operator<<(ScopedState<T> &to, const ScopedState<Y> &from)
        {
            copyStateData(to.getSpace(), to.get(), from.getSpace(), from.get());
            return to;
        }

        /** \brief This is a fancy version of the assignment
            operator. It is a partial assignment, in some sense. The
            difference is that if the states are part of compound
            state spaces, the data is copied from \e from to \e to on a
            component by component basis. State spaces are matched by
            name. If the state space for \e to contains any subspace
            whose name matches any subspace of the state space for \e
            from, the corresponding state components are copied. */
        template <class T, class Y>
        inline const ScopedState<T> &operator>>(const ScopedState<T> &from, ScopedState<Y> &to)
        {
            copyStateData(to.getSpace(), to.get(), from.getSpace(), from.get());
            return from;
        }

        /** \brief Given state \e a from state space A and state \e b
            from state space B, construct a state from state space A
            + B. The resulting state contains all the information from
            the input states (the states are concatenated). */
        template <class T, class Y>
        inline const ScopedState<> operator^(const ScopedState<T> &a, const ScopedState<Y> &b)
        {
            ScopedState<> r(a.getSpace() + b.getSpace());
            return r << a << b;
        }

        /** @} */

        template <class T>
        const ScopedState<> ScopedState<T>::operator[](const StateSpacePtr &s) const
        {
            ScopedState<> r(s);
            return r << *this;
        }

        /** \brief Shared pointer to a ScopedState<> */
        using ScopedStatePtr = std::shared_ptr<ScopedState<>>;
    }  // namespace base
}  // namespace ompl

#endif
