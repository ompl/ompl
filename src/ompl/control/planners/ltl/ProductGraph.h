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

#ifndef OMPL_CONTROL_PLANNERS_LTL_PRODUCTGRAPH_
#define OMPL_CONTROL_PLANNERS_LTL_PRODUCTGRAPH_

#include "ompl/base/State.h"
#include "ompl/control/planners/ltl/Automaton.h"
#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/util/ClassForward.h"
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <map>
#include <ostream>
#include <vector>

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::ProductGraph */
        OMPL_CLASS_FORWARD(ProductGraph);
        /// @endcond

        /** \class ompl::control::ProductGraphPtr
            \brief A shared pointer wrapper for ompl::control::ProductGraph */

        /** \brief A ProductGraph represents the weighted, directed, graph-based
            Cartesian product of a PropositionalDecomposition object,
            an Automaton corresponding to a co-safe LTL specification,
            and an Automaton corresponding to a safe LTL specification. */
        class ProductGraph
        {
        public:
            class State;

            /// @cond IGNORE
            /** \brief Hash function for State to be used in std::unordered_map */
            struct HashState
            {
                std::size_t operator()(const State &s) const;
            };
            /// @endcond

            /** \brief A State of a ProductGraph represents a vertex in the graph-based
                Cartesian product represented by the ProductGraph.
                A State is simply a tuple consisting of a PropositionalDecomposition region,
                a co-safe Automaton state, and a safe Automaton state. */
            class State
            {
                friend class ProductGraph;

            public:
                /** \brief Creates a State without any assigned PropositionalDecomposition
                    region or Automaton states. All of these values are initialized to -1. */
                State() = default;

                /** \brief Basic copy constructor for State. */
                State(const State &s) = default;

                /** \brief Returns whether this State is equivalent to a given State,
                    by comparing their PropositionalDecomposition regions and
                    Automaton states. */
                bool operator==(const State &s) const;

                /** \brief Returns whether this State is valid.
                    A State is valid if and only if none of its Automaton states
                    are dead states (a dead state has value -1). */
                bool isValid() const;

                friend struct HashState;

                /** \brief Helper function to print this State to a given output stream. */
                friend std::ostream &operator<<(std::ostream &out, const State &s);

                /** \brief Returns this State's PropositionalDecomposition region component. */
                int getDecompRegion() const;

                /** \brief Returns this State's co-safe Automaton state component. */
                int getCosafeState() const;

                /** \brief Returns this State's safe Automaton state component. */
                int getSafeState() const;

            private:
                int decompRegion{-1};
                int cosafeState{-1};
                int safeState{-1};
            };

            /** \brief Initializes a ProductGraph with a given PropositionalDecomposition,
                co-safe Automaton, and safe Automaton. */
            ProductGraph(PropositionalDecompositionPtr decomp, AutomatonPtr cosafetyAut, AutomatonPtr safetyAut);

            /** \brief Initializes an abstraction with a given propositional
                decomposition and cosafety automaton. The safety automaton is
                set to be one that always accepts. */
            ProductGraph(const PropositionalDecompositionPtr &decomp, AutomatonPtr cosafetyAut);

            ~ProductGraph();

            /** \brief Returns the PropositionalDecomposition contained
                within this ProductGraph. */
            const PropositionalDecompositionPtr &getDecomp() const;

            /** \brief Returns the co-safe Automaton contained
                within this ProductGraph. */
            const AutomatonPtr &getCosafetyAutom() const;

            /** \brief Returns the safe Automaton contained
                within this ProductGraph. */
            const AutomatonPtr &getSafetyAutom() const;

            /** \brief Returns a shortest-path sequence of ProductGraph states, beginning with
                a given initial State and ending with a State for which
                the corresponding safety Automaton state is accepting,
                and the corresponding co-safety Automaton state is as close as possible
                to an accepting state given the adjacency properties of the
                PropositionalDecomposition.
                Dijkstra's shortest-path algorithm is used to compute the path with
                the given edge-weight function. */
            std::vector<State *> computeLead(State *start, const std::function<double(State *, State *)> &edgeWeight);

            /** \brief Clears all memory belonging to this ProductGraph. */
            void clear();

            /** \brief Constructs this ProductGraph beginning with a given initial State,
                using a breadth-first search. Accepts an optional initialization method,
                which will be called exactly once on each State (including the given initial State)
                that is added to the ProductGraph.
                The default argument for the initialization method is a no-op method. */
            void buildGraph(State *start, const std::function<void(State *)> &initialize = [](State *){});

            /** \brief Returns whether the given State is an accepting State
                in this ProductGraph.
                We call a State accepting if its safety Automaton state component
                is accepting, and its co-safety Automaton state component
                is as close as possible to an accepting state in the co-safety Automaton
                given the adjacency properties of the PropositionalDecomposition. */
            bool isSolution(const State *s) const;

            /** \brief Returns the initial State of this ProductGraph. */
            State *getStartState() const;

            /** \brief Helper method to return the volume of the PropositionalDecomposition
                region corresponding to the given ProductGraph State. */
            double getRegionVolume(const State *s);

            /** \brief Helper method to return the distance from a given State's
                co-safety state to an accepting state in the co-safety Automaton. */
            int getCosafeAutDistance(const State *s) const;

            /** \brief Helper method to return the distance from a given State's
                safety state to an accepting state in the safety Automaton. */
            int getSafeAutDistance(const State *s) const;

            /** \brief Returns a ProductGraph State with initial co-safety and safety
                Automaton states, and the PropositionalDecomposition region that contains
                a given base::State. */
            State *getState(const base::State *cs) const;

            /** \brief Returns a ProductGraph State with given co-safety and safety
                Automaton states, and the PropositionalDecomposition region that contains
                a given base::State. */
            State *getState(const base::State *cs, int cosafe, int safe) const;

            /** \brief Returns a ProductGraph State with a given PropositionalDecomposition region.
                The co-safety and safety Automaton states are calculated using a given parent
                ProductGraph State and the decomposition region. */
            State *getState(const State *parent, int nextRegion) const;

            /** \brief Returns a ProductGraph state with the PropositionalDecomposition region
                that contains a given base::State.
                The co-safety and safety Automaton states are calculated using a given parent
                ProductGraph State and the decomposition region. */
            State *getState(const State *parent, const base::State *cs) const;

            /** \brief Returns the ProductGraph state corresponding to the given region,
                co-safety state, and safety state. */
            State *getState(int region, int cosafe, int safe) const
            {
                State s;
                s.decompRegion = region;
                s.cosafeState = cosafe;
                s.safeState = safe;
                State *&ret = stateToPtr_[s];
                if (ret == nullptr)
                    ret = new State(s);
                return ret;
            }

        protected:
            struct Edge
            {
                double cost;
            };

            using GraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, State *, Edge>;
            using Vertex = boost::graph_traits<GraphType>::vertex_descriptor;
            using VertexIter = boost::graph_traits<GraphType>::vertex_iterator;
            using VertexIndexMap = boost::property_map<GraphType, boost::vertex_index_t>::type;
            using EdgeIter = boost::graph_traits<GraphType>::edge_iterator;

            PropositionalDecompositionPtr decomp_;
            AutomatonPtr cosafety_;
            AutomatonPtr safety_;
            GraphType graph_;
            State *startState_;
            std::vector<State *> solutionStates_;

            /* Only one State pointer will be allocated for each possible State
               in the ProductGraph. There will exist situations in which
               all we have are the component values (region, automaton states)
               of a State and we want the actual State pointer.
               We use this map to access it. */
            mutable std::unordered_map<State, State *, HashState> stateToPtr_;

            /* Map from State pointer to the index of the corresponding vertex
               in the graph. */
            std::unordered_map<State *, int> stateToIndex_;
        };
    }
}
#endif
