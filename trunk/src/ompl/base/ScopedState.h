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

namespace ompl
{
    namespace base
    {

        /** \brief Definition of a scoped state.

            This class allocates a state of a desired type using the
            allocation mechanism of the manifold the state is part
            of. The state is then freed when the instance goes out of
            scope using the corresponding free mechanism. */
        template<class T = StateManifold>
        class ScopedState
        {
            /** \brief Make sure the type we are allocating is indeed from a manifold */
            BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));

            /** \brief Make sure the type we are allocating is indeed a state */
            BOOST_CONCEPT_ASSERT((boost::Convertible<typename T::StateType*, State*>));

        public:

            /** \brief The type of the contained state */
            typedef typename T::StateType StateType;

            /** \brief Given the space that we are working with,
                allocate a state from the corresponding
                manifold.  */
            explicit
            ScopedState(const SpaceInformationPtr &si) : manifold_(si->getStateManifold())
            {
                State *s = manifold_->allocState();

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType*>(s);
            }

            /** \brief Given the manifold that we are working with,
                allocate a state. */
            explicit
            ScopedState(const StateManifoldPtr &manifold) : manifold_(manifold)
            {
                State *s = manifold_->allocState();

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType*>(s);
            }

            /** \brief Copy constructor */
            ScopedState(const ScopedState<T> &other) : manifold_(other.getManifold())
            {
                State *s = manifold_->allocState();
                state_ = static_cast<StateType*>(s);
                manifold_->copyState(s, static_cast<const State*>(other.get()));
            }

            /** \brief Copy constructor that allows instantiation from states of other type */
            template<class O>
            ScopedState(const ScopedState<O> &other) : manifold_(other.getManifold())
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateManifold*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                State *s = manifold_->allocState();
                state_ = static_cast<StateType*>(s);
                manifold_->copyState(s, static_cast<const State*>(other.get()));
            }

            /** \brief Given the manifold that we are working with,
                allocate a state and fill that state with a given value. */
            ScopedState(const StateManifoldPtr &manifold, const State *state) : manifold_(manifold)
            {
                State *s = manifold_->allocState();
                manifold_->copyState(s, state);

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType*>(s);
            }

            /** \brief Free the memory of the internally allocated state */
            ~ScopedState(void)
            {
                manifold_->freeState(state_);
            }

            /** \brief Get the manifold that the state corresponds to */
            const StateManifoldPtr& getManifold(void) const
            {
                return manifold_;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const ScopedState<T> &other)
            {
                if (&other != this)
                {
                    manifold_->freeState(state_);
                    manifold_ = other.getManifold();

                    State *s = manifold_->allocState();
                    state_ = static_cast<StateType*>(s);
                    manifold_->copyState(s, static_cast<const State*>(other.get()));
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const State *other)
            {
                if (other != static_cast<State*>(state_))
                {
                    // ideally, we should use a dynamic_cast and throw an
                    // exception in case other does not cast to
                    // const StateType*. However, RTTI may not be
                    // available across shared library boundaries, so we
                    // do not use it

                    manifold_->copyState(static_cast<State*>(state_), other);
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const State &other)
            {
                if (&other != static_cast<State*>(state_))
                {
                    // ideally, we should use a dynamic_cast and throw an
                    // exception in case &other does not cast to
                    // const StateType*. However, RTTI may not be
                    // available across shared library boundaries, so we
                    // do not use it

                    manifold_->copyState(static_cast<State*>(state_), &other);
                }
                return *this;
            }

            /** \brief Assignment operator that allows conversion of states */
            template<class O>
            ScopedState<T>& operator=(const ScopedState<O> &other)
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateManifold*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                if (reinterpret_cast<const void*>(&other) != reinterpret_cast<const void*>(this))
                {
                    manifold_->freeState(state_);
                    manifold_ = other.getManifold();

                    State *s = manifold_->allocState();
                    state_ = static_cast<StateType*>(s);
                    manifold_->copyState(s, static_cast<const State*>(other.get()));
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const std::vector<double> &reals)
            {
                manifold_->copyFromReals(state_, reals);
                return *this;
            }

            /** \brief Checks equality of two states */
            template<class O>
            bool operator==(const ScopedState<O> &other) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateManifold*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                return manifold_->equalStates(static_cast<const State*>(state_), static_cast<const State*>(other.get()));
            }

            /** \brief Checks equality of two states */
            template<class O>
            bool operator!=(const ScopedState<O> &other) const
            {
                return !(*this == other);
            }

            /** \brief Extract a state that corresponds to the
                components in manifold \e m. Those components will
                have the same value as the current state (only the
                ones included in the current state; others will be
                uninitialised) */
            ScopedState<> operator[](const StateManifoldPtr &m) const;

            /** \brief Set this state to a random value (uniform) */
            void random(void)
            {
                if (!sampler_)
                    sampler_ = manifold_->allocStateSampler();
                sampler_->sampleUniform(state_);
            }

            /** \brief Return the real values corresponding to this
                state. If a conversion is not possible, an exception
                is thrown.*/
            std::vector<double> reals(void) const
            {
                std::vector<double> r;
                manifold_->copyToReals(state_, r);
                return r;
            }

            /** \brief Print this state to a stream */
            void print(std::ostream &out = std::cout) const
            {
                manifold_->printState(state_, out);
            }

            /** \brief De-references to the contained state */
            StateType& operator*(void) const
            {
                return *state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType* operator->(void) const
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType* get(void) const
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType* operator()(void) const
            {
                return state_;
            }

        private:

            StateManifoldPtr         manifold_;
            ManifoldStateSamplerPtr  sampler_;
            StateType               *state_;
        };

        /** \addtogroup stateAndManifoldOperators Operators for States and Manifolds

           These operators are intended to simplify code that
           manipulates states and manifolds. They rely on the fact
           that manifolds have unique names. Here are some examples
           for using these operators:
          \code
           // Assume X, Y, Z, W are state manifold instances, none of
           // which inherits from ompl::base::CompoundStateManifold.
           // Denote a compound manifold as C[...], where "..." is the
           // list of submanifolds.

           ompl::base::StateManifoldPtr X;
           ompl::base::StateManifoldPtr Y;
           ompl::base::StateManifoldPtr Z;
           ompl::base::StateManifoldPtr W;

           // the following line will construct a manifold C1 = C[X, Y]
           ompl::base::StateManifoldPtr C1 = X + Y;

           // the following line will construct a manifold C2 = C[X, Y, Z]
           ompl::base::StateManifoldPtr C2 = C1 + Z;

           // the following line will leave C2 as C[X, Y, Z]
           ompl::base::StateManifoldPtr C2 = C1 + C2;

           // the following line will construct a manifold C2 = C[X, Y, Z, W]
           ompl::base::StateManifoldPtr C2 = C2 + W;

           // the following line will construct a manifold C3 = C[X, Z, Y]
           ompl::base::StateManifoldPtr C3 = X + Z + Y;

           // the following line will construct a manifold C4 = C[Z, W]
           ompl::base::StateManifoldPtr C4 = C2 - C1;

           // the following line will construct a manifold C5 = W
           ompl::base::StateManifoldPtr C5 = C2 - C3;

           // the following line will construct an empty manifold C6 = C[]
           ompl::base::StateManifoldPtr C6 = X - X;

           // the following line will construct an empty manifold C7 = Y
           ompl::base::StateManifoldPtr C7 = Y + C6;
          \endcode
           These manifolds can be used when operating with states:
          \code
           ompl::base::ScopedState<> sX(X);
           ompl::base::ScopedState<> sXY(X + Y);
           ompl::base::ScopedState<> sY(Y);
           ompl::base::ScopedState<> sZX(Z + X);
           ompl::base::ScopedState<> sXZW(X + Z + W);

           // the following line will copy the content of the state sX to
           // the corresponding locations in sXZW. The components of the state
           // corresponding to the Z and W manifolds are not touched
           sX >> sXZW;

           // the following line will initialize the X component of sXY with
           // the X component of sXZW;
           sXY << sXZW;

           // the following line will initialize both components of sZX, using
           // the X and Z components of sXZW;
           sZX << sXZW;

           // the following line compares the concatenation of states sX and sY with sXY
           // the concatenation will automatically construct the manifold X + Y and a state
           // from that manifold containing the information from sX and sY. Since sXY is
           // constructed from the manifold X + Y, the two are comparable.
           bool eq = (sX ^ sY) == sXY;
          \endcode
            @{
         */

        /** \brief Overload stream output operator. Calls ompl::base::StateManifold::printState() */
        inline
        std::ostream& operator<<(std::ostream &out, const ScopedState<> &state)
        {
            state.print(out);
            return out;
        }

        /// @cond IGNORE

        // workhorse for the << and >> operators defined on states
        int __private_insertStateData(const StateManifoldPtr &destM, State *dest,
                                      const StateManifoldPtr &sourceM, const State *source);

        /// @endcond

        /** \brief This is a fancy version of the assingment
            operator. It is a partial assignment, in some sense. The
            difference is that if the states are part of compound
            manifolds, the data is copied from \e from to \e to on a
            component by component basis. Manifolds are matched by
            name. If the manifold for \e to contains any sub-manifold
            whose name matches any sub-manifold of the manifold for \e
            from, the corresponding state components are copied. */
        template<class T, class Y>
        inline
        ScopedState<T>& operator<<(ScopedState<T> &to, const ScopedState<Y> &from)
        {
            __private_insertStateData(to.getManifold(), to.get(), from.getManifold(), from.get());
            return to;
        }

        /** \brief This is a fancy version of the assingment
            operator. It is a partial assignment, in some sense. The
            difference is that if the states are part of compound
            manifolds, the data is copied from \e from to \e to on a
            component by component basis. Manifolds are matched by
            name. If the manifold for \e to contains any sub-manifold
            whose name matches any sub-manifold of the manifold for \e
            from, the corresponding state components are copied. */
        template<class T, class Y>
        inline
        const ScopedState<T>& operator>>(const ScopedState<T> &from, ScopedState<Y> &to)
        {
            __private_insertStateData(to.getManifold(), to.get(), from.getManifold(), from.get());
            return from;
        }

        /** \brief Given state \e a from manifold A and state \e b
            from manifold B, construct a state from manifold A
            + B. The resulting state contains all the information from
            the input states (the states are concatenated). */
        template<class T, class Y>
        inline
        ScopedState<> operator^(const ScopedState<T> &a, const ScopedState<Y> &b)
        {
            ScopedState<> r(a.getManifold() + b.getManifold());
            return r << a << b;
        }

        /** @} */

        template<class T>
        ScopedState<> ScopedState<T>::operator[](const StateManifoldPtr &m) const
        {
            ScopedState<> r(m);
            return r << *this;
        }

        /** \brief Shared pointer to a ScopedState<> */
        typedef boost::shared_ptr< ScopedState<> > ScopedStatePtr;
    }
}

#endif
