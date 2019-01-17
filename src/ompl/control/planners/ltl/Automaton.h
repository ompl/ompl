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

/* Author: Matt Maly, Keliang He */

#ifndef OMPL_CONTROL_PLANNERS_LTL_AUTOMATON_
#define OMPL_CONTROL_PLANNERS_LTL_AUTOMATON_

#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/ClassForward.h"
#include "ompl/config.h"
#include <unordered_map>
#include <limits>
#include <ostream>
#include <vector>

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::Automaton */
        OMPL_CLASS_FORWARD(Automaton);
        /// @endcond

        /** \class ompl::control::AutomatonPtr
            \brief A shared pointer wrapper for ompl::control::Automaton */

        /** \brief A class to represent a deterministic finite automaton,
            each edge of which corresponds to a World.
            A system trajectory, by way of project() and worldAtRegion()
            in PropositionalDecomposition, determines a sequence of Worlds,
            which are read by an Automaton to determine whether a trajectory
            satisfies a given specification.

            An automaton is meant to be run in a read-only fashion, i.e.,
            it does not keep track of an internal state and can be thought of
            as a lookup table. */
        class Automaton
        {
        public:
            /** \brief Each automaton state has a transition map, which maps from a
                World to another automaton state.
                A set \f$P\f$ of true propositions correponds to the formula
                \f$\bigwedge_{p\in P} p\f$. */
            struct TransitionMap
            {
                /** \brief Returns the automaton state corresponding to a given
                    World in this transition map.
                    Returns -1 if no such transition exists. */
                int eval(const World &w) const;

                TransitionMap &operator=(const TransitionMap &tm) = default;

                mutable std::unordered_map<World, unsigned int> entries;
            };

            /** \brief Creates an automaton with a given number of propositions and states. */
            Automaton(unsigned int numProps, unsigned int numStates = 0);

#if OMPL_HAVE_SPOT
            /** \brief Creates an automaton with a given number of propositions from an LTL
                formula using Spot. By default, formulas are assumed to be co-safe LTL
                formulas. If isCosafe is set to false, the formula is assumed to be a safe
                LTL formula. */
            Automaton(unsigned numProps, std::string formula, bool isCosafe = true);
#endif

            /** \brief Adds a new state to the automaton and returns an ID for it. */
            unsigned int addState(bool accepting = false);

            /** \brief Sets the accepting status of a given state. */
            void setAccepting(unsigned int s, bool a);

            /** \brief Returns whether a given state of the automaton is accepting. */
            bool isAccepting(unsigned int s) const;

            /** \brief Sets the start state of the automaton. */
            void setStartState(unsigned int s);

            /** \brief Returns the start state of the automaton.
                Returns -1 if no start state has been set. */
            int getStartState() const;

            /** \brief Adds a given transition to the automaton. */
            void addTransition(unsigned int src, const World &w, unsigned int dest);

            /** \brief Runs the automaton from its start state, using
                the values of propositions from a given sequence of Worlds.
                Returns false if and only if the result is a nonexistent state
                (i.e., if and only if there does not exist an extension to trace
                that will lead it to an accepting state). */
            bool run(const std::vector<World> &trace) const;

            /** \brief Runs the automaton for one step from the given state,
                using the values of propositions from a given World.
                Returns the resulting state, or -1 if the result is a nonexistent state. */
            int step(int state, const World &w) const;

            /** \brief Returns the outgoing transition map for a given automaton state. */
            TransitionMap &getTransitions(unsigned int src);

            /** \brief Returns the number of states in this automaton. */
            unsigned int numStates() const;

            /** \brief Returns the number of transitions in this automaton. */
            unsigned int numTransitions() const;

            /** \brief Returns the number of propositions used by this automaton. */
            unsigned int numProps() const;

            /** \brief Prints the automaton to a given output stream, in Graphviz dot format. */
            void print(std::ostream &out) const;

            /** \brief Returns the shortest number of transitions from a given state to
                an accepting state. */
            unsigned int distFromAccepting(unsigned int s) const;

            /** \brief Returns a single-state automaton that accepts on all inputs. */
            static AutomatonPtr AcceptingAutomaton(unsigned int numProps);

            /** \brief Helper function to return a coverage automaton.
                Assumes all propositions are mutually exclusive. */
            static AutomatonPtr CoverageAutomaton(unsigned int numProps, const std::vector<unsigned int> &covProps);

            /** \brief Helper function to return a sequence automaton.
                Assumes all propositions are mutually exclusive. */
            static AutomatonPtr SequenceAutomaton(unsigned int numProps, const std::vector<unsigned int> &seqProps);

            /** \brief Helper function to return a disjunction automaton,
                which accepts when one of the given propositions becomes true. */
            static AutomatonPtr DisjunctionAutomaton(unsigned int numProps, const std::vector<unsigned int> &disjProps);

            /** \brief Returns an avoidance automaton, which rejects when any one of the
                given list of propositions becomes true. Accepts otherwise. */
            static AutomatonPtr AvoidanceAutomaton(unsigned int numProps, const std::vector<unsigned int> &avoidProps);

            /** \brief Helper function to return a coverage automaton
                over propositions from 0 to numProps-1.
                Assumes all propositions are mutually exclusive. */
            static AutomatonPtr CoverageAutomaton(unsigned int numProps);

            /** \brief Helper function to return a sequence automaton
                over propositions from 0 to numProps-1, in that order.
                Assumes all propositions are mutually exclusive. */
            static AutomatonPtr SequenceAutomaton(unsigned int numProps);

            /** \brief Helper function to return a disjunction automaton,
                which accepts when one of the given propositions in [0,numProps-1] becomes true. */
            static AutomatonPtr DisjunctionAutomaton(unsigned int numProps);

        protected:
            unsigned int numProps_;
            unsigned int numStates_;
            int startState_{-1};
            std::vector<bool> accepting_;
            std::vector<TransitionMap> transitions_;
            mutable std::vector<unsigned int> distances_;
        };
    }
}
#endif
