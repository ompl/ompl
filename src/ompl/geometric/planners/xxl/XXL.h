/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_XXL_XXL_
#define OMPL_GEOMETRIC_PLANNERS_XXL_XXL_

#include <thread>
#include <unordered_map>
#include "ompl/util/Hash.h"
#include "ompl/datastructures/AdjacencyList.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/xxl/XXLDecomposition.h"


namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gXXL

        \ref gXXL "XXL" is a probabilistically complete sampling-based algorithm designed to plan the motions of high-dimensional mobile manipulators and related platforms. Using a novel sampling and connection strategy that guides a set of points mapped on the robot through the workspace, XXL scales to realistic manipulator platforms with dozens of joints by focusing the search of the robot's configuration space to specific degrees-of-freedom that affect motion in particular portions of the workspace. Simulated planning scenarios with the Robonaut2 platform and planar kinematic chains confirm that XXL exhibits competitive solution times relative to many existing works while obtaining execution-quality solution paths. Solutions from XXL are of comparable quality to costaware methods even though XXL does not explicitly optimize over any particular criteria, and are computed in an order of magnitude less time.

        @par Associated publication:
        R. Luna, M. Moll, J. Badger, and L. E. Kavraki,
        A Scalable Motion Planner for High-Dimensional Kinematic Systems,
        <em>Intl. J. of Robotics Research</em>, vol. 39, issue 4, pp. 361-388, Mar. 2020.
        DOI: [10.1177/0278364919890408](http://dx.doi.org/10.1177/0278364919890408)<br>
        [[PDF]](http://www.kavrakilab.org/publications/luna2020a-scalable-motion-planner-for-high-dimensional.pdf)
        */
        class XXL : public base::Planner
        {
        public:
            // Standard planner constructor.  Must call setDecomposition before calling solve()
            XXL(const base::SpaceInformationPtr &si);

            // Initialize HiLo with the given decomposition
            XXL(const base::SpaceInformationPtr &si, const XXLDecompositionPtr &decomp);

            virtual ~XXL();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void setup();

            void setDecomposition(const XXLDecompositionPtr &decomp);

            double getRandWalkRate() const
            {
                return rand_walk_rate_;
            }
            void setRandWalkRate(double rate)
            {
                if (rate < 0.0 || rate > 1.0)
                    throw;
                rand_walk_rate_ = rate;
            }

        protected:
            // Quickly insert, check membership, and grab a unique integer from a range [0, max)
            class PerfectSet
            {
            public:
                PerfectSet(std::size_t max)
                {
                    exists.assign(max, false);
                    elements.reserve(max / 10);  // reserve, but do not "allocate" the space
                }

                std::size_t numElements() const
                {
                    return elements.size();
                }

                bool isElement(unsigned int val) const
                {
                    if (val >= (unsigned int)exists.size())
                        return false;
                    return exists[val];
                }

                bool addElement(unsigned int val)
                {
                    if (val >= (unsigned int)exists.size() || exists[val])
                        return false;

                    elements.push_back(val);
                    exists[val] = true;
                    return true;
                }

                int getElement(std::size_t idx) const
                {
                    return elements[idx];
                }

            protected:
                std::vector<bool> exists;
                std::vector<unsigned int> elements;
            };

            struct Motion
            {
                base::State *state;
                std::vector<int> levels;
                int index;
            };

            struct Region
            {
                std::vector<int> allMotions;
                std::vector<int> motionsInTree;  // subset of allMotions that are connected to the tree
            };

            class Layer
            {
            public:
                Layer(int _id, int numRegions, int lvl, Layer *_parent)
                  : regions(numRegions)
                  , weights(numRegions, 0.5)
                  , exterior(numRegions, true)
                  , connections(numRegions, 0)
                  , selections(numRegions, 0)
                  , leads(numRegions, 0)
                  , goalStates(numRegions, std::vector<int>())
                  , connectionPoints(numRegions)
                  , regionGraph(new AdjacencyList(numRegions))
                  , level(lvl)
                  , id(_id)
                  , parent(_parent)
                {
                }

                ~Layer()
                {
                    delete regionGraph;
                    for (auto &sublayer : sublayers)
                        delete sublayer;
                }

                size_t numRegions() const
                {
                    return regions.size();
                }

                int getLevel() const
                {
                    return level;
                }

                Layer *getParent() const
                {
                    return parent;
                }

                int getID() const
                {
                    return id;
                }

                Region &getRegion(int r)
                {
                    if (r < 0 || r >= (int)regions.size())
                    {
                        OMPL_ERROR("Requested region %d, but there are only %lu regions", r, regions.size());
                        throw ompl::Exception("Region out of bounds");
                    }

                    return regions[r];
                }

                const Region &getRegion(int r) const
                {
                    if (r < 0 || r >= (int)regions.size())
                    {
                        OMPL_ERROR("Requested region %d, but there are only %lu regions", r, regions.size());
                        throw ompl::Exception("Region out of bounds");
                    }

                    return regions[r];
                }

                const std::vector<double> &getWeights() const
                {
                    return weights;
                }

                std::vector<double> &getWeights()
                {
                    return weights;
                }

                const std::vector<bool> &getExterior() const
                {
                    return exterior;
                }

                std::vector<bool> &getExterior()
                {
                    return exterior;
                }

                const std::vector<int> &getConnections() const
                {
                    return connections;
                }

                const std::vector<int> &getSelections() const
                {
                    return selections;
                }

                const std::vector<std::vector<int>> &getGoalStates() const
                {
                    return goalStates;
                }

                const std::vector<int> &getGoalStates(int reg) const
                {
                    return goalStates[reg];
                }

                size_t numGoalStates() const
                {
                    return totalGoalStates;
                }

                size_t numGoalStates(int reg) const
                {
                    return goalStates[reg].size();
                }

                void addGoalState(int reg, int id)
                {
                    goalStates[reg].push_back(id);
                    totalGoalStates++;
                }

                AdjacencyList &getRegionGraph()
                {
                    return *regionGraph;
                }

                const AdjacencyList &getRegionGraph() const
                {
                    return *regionGraph;
                }

                Layer *getSublayer(int l)
                {
                    return sublayers[l];
                }

                const Layer *getSublayer(int l) const
                {
                    return sublayers[l];
                }

                void allocateSublayers()
                {
                    if (sublayers.size())
                        throw;

                    for (size_t i = 0; i < regions.size(); ++i)
                        sublayers.push_back(new Layer(i, regions.size(), level + 1, this));
                }

                bool hasSublayers()
                {
                    return !sublayers.empty();
                }

                void selectRegion(int r, int count = 1)
                {
                    // numSelections++;
                    // selections[r]++;
                    numSelections += count;
                    selections[r] += count;
                }

                void connectRegion(int r)
                {
                    numConnections++;
                    connections[r]++;
                    connectionPoints.addElement(r);
                }

                int totalSelections() const
                {
                    return numSelections;
                }

                int totalConnections() const
                {
                    return numConnections;
                }

                int connectibleRegions() const
                {
                    return connectionPoints.numElements();
                }

                int connectibleRegion(int idx) const
                {
                    return connectionPoints.getElement(idx);
                }

                int leadAppearances(int idx) const
                {
                    return leads[idx];
                }

                int numLeads() const
                {
                    return numTotalLeads;
                }

                void markLead(const std::vector<int> &lead)
                {
                    numTotalLeads++;
                    for (size_t i = 0; i < lead.size(); ++i)
                        leads[lead[i]]++;
                }

            protected:
                std::vector<Region> regions;   // The list of regions in this layer
                std::vector<double> weights;   // Weight for each region
                std::vector<bool> exterior;    // Exterior status for the regions in this layer
                std::vector<int> connections;  // Number of times the search has tried internal connections in this
                                               // region
                std::vector<int> selections;   // Number of times the search has selected this region for expansion
                std::vector<int> leads;          // Number of times each region has appeared in a lead
                std::vector<std::vector<int>> goalStates;  // A list of goal states in each region
                PerfectSet connectionPoints;  // The set of regions we have tried to do internal connections on

                AdjacencyList *regionGraph;      // The connectivity of regions in this layer
                std::vector<Layer *> sublayers;  // The layers descending from this layer
                int level;                       // The level of this layer in the hierarchy (0 is top)
                int numSelections{0};            // The total number of selections in this layer
                int numConnections{0};           // The total number of internal connection attempts in this layer
                int id;
                int totalGoalStates{0};  // The total number of goal states in this layer
                int numTotalLeads{0};
                Layer *parent;
            };

            void freeMemory();
            void allocateLayers(Layer *layer);

            void updateRegionConnectivity(const Motion *m1, const Motion *m2, int layer);
            Layer *getLayer(const std::vector<int> &regions, int layer);

            int addState(const base::State *state);
            int addThisState(base::State *state);  // add state using this state memory, no copy
            int addGoalState(const base::State *state);
            int addStartState(const base::State *state);

            // Update the various statistics for the regions and their subregions in the vector
            void updateRegionProperties(const std::vector<int> &regions);
            // Update the various statistics for the region specified
            void updateRegionProperties(Layer *layer, int region);

            // Sample states uniformly at random in the given layer until ptc is triggered
            void sampleStates(Layer *layer, const ompl::base::PlannerTerminationCondition &ptc);
            bool sampleAlongLead(Layer *layer, const std::vector<int> &lead,
                                 const ompl::base::PlannerTerminationCondition &ptc);

            int steerToRegion(Layer *layer, int from, int to);
            int expandToRegion(Layer *layer, int from, int to, bool useExisting = false);

            bool feasibleLead(Layer *layer, const std::vector<int> &lead,
                              const ompl::base::PlannerTerminationCondition &ptc);
            bool connectLead(Layer *layer, const std::vector<int> &lead, std::vector<int> &candidateRegions,
                             const ompl::base::PlannerTerminationCondition &ptc);
            void connectRegion(Layer *layer, int region, const base::PlannerTerminationCondition &ptc);
            void connectRegions(Layer *layer, int r1, int r2, const base::PlannerTerminationCondition &ptc,
                                bool all = false);

            // Compute a new lead in the given decomposition layer from start to goal
            void computeLead(Layer *layer, std::vector<int> &lead);

            // Search for a solution path in the given layer
            bool searchForPath(Layer *layer, const ompl::base::PlannerTerminationCondition &ptc);

            // Return a list of neighbors and the edge weights from rid
            void getNeighbors(int rid, const std::vector<double> &weights,
                              std::vector<std::pair<int, double>> &neighbors) const;

            // Shortest (weight) path from r1 to r2
            bool shortestPath(int r1, int r2, std::vector<int> &path, const std::vector<double> &weights);

            // Compute a path from r1 to r2 via a random walk
            bool randomWalk(int r1, int r2, std::vector<int> &path);

            void getGoalStates();
            // Thread that gets us goal states
            void getGoalStates(const base::PlannerTerminationCondition &ptc);

            bool constructSolutionPath();

            bool isStartState(int idx) const;
            bool isGoalState(int idx) const;

            void writeDebugOutput() const;

            // Root layer of the decomposition data
            Layer *topLayer_{nullptr};

            // Raw storage for all motions explored during search
            std::vector<Motion *> motions_;
            // Indexes into motions_ for start states
            std::vector<int> startMotions_;
            // Index into motions_ for goal states
            std::vector<int> goalMotions_;
            // The number of goal states in each decomposition cell
            std::unordered_map<std::vector<int>, int> goalCount_;

            base::State *xstate_;

            // The number of states in realGraph that have verified edges in the graph
            unsigned int statesConnectedInRealGraph_;

            unsigned int maxGoalStatesPerRegion_;
            unsigned int maxGoalStates_;

            // Random number generator
            RNG rng_;

            base::StateSamplerPtr sampler_;

            // A decomposition of the search space
            XXLDecompositionPtr decomposition_;

            // A lazily constructed graph where edges between states have not been collision checked
            AdjacencyList lazyGraph_;
            // The verified graph where all edges have been collision checked.  An edge in this graph
            // should not exist in lazyGraph
            AdjacencyList realGraph_;

            // Variable for the goal state sampling thread
            bool kill_{false};

            // Scratch space for shortest path computation
            std::vector<int> predecessors_;
            std::vector<bool> closedList_;

            double rand_walk_rate_{-1.0};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
