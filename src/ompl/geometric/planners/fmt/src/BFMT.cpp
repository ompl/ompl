
#include <boost/math/constants/constants.hpp>
#include <boost/math/distributions/binomial.hpp>
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/tools/config/SelfConfig.h>

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/fmt/BFMT.h>

#include <fstream>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace geometric
    {
        BFMT::BFMT(const base::SpaceInformationPtr &si)
          : base::Planner(si, "BFMT")
          , freeSpaceVolume_(si_->getStateSpace()->getMeasure())  // An upper bound on the free space volume is the
                                                                  // total space volume; the free fraction is estimated
                                                                  // in sampleFree
        {
            specs_.approximateSolutions = false;
            specs_.directed = false;

            ompl::base::Planner::declareParam<unsigned int>("num_samples", this, &BFMT::setNumSamples,
                                                            &BFMT::getNumSamples, "10:10:1000000");
            ompl::base::Planner::declareParam<double>("radius_multiplier", this, &BFMT::setRadiusMultiplier,
                                                      &BFMT::getRadiusMultiplier, "0.1:0.05:50.");
            ompl::base::Planner::declareParam<bool>("nearest_k", this, &BFMT::setNearestK, &BFMT::getNearestK, "0,1");
            ompl::base::Planner::declareParam<bool>("balanced", this, &BFMT::setExploration, &BFMT::getExploration,
                                                    "0,1");
            ompl::base::Planner::declareParam<bool>("optimality", this, &BFMT::setTermination, &BFMT::getTermination,
                                                    "0,1");
            ompl::base::Planner::declareParam<bool>("heuristics", this, &BFMT::setHeuristics, &BFMT::getHeuristics,
                                                    "0,1");
            ompl::base::Planner::declareParam<bool>("cache_cc", this, &BFMT::setCacheCC, &BFMT::getCacheCC, "0,1");
            ompl::base::Planner::declareParam<bool>("extended_fmt", this, &BFMT::setExtendedFMT, &BFMT::getExtendedFMT,
                                                    "0,1");
        }

        ompl::geometric::BFMT::~BFMT()
        {
            freeMemory();
        }

        void BFMT::setup()
        {
            if (pdef_)
            {
                /* Setup the optimization objective. If no optimization objective was
                specified, then default to optimizing path length as computed by the
                distance() function in the state space */
                if (pdef_->hasOptimizationObjective())
                    opt_ = pdef_->getOptimizationObjective();
                else
                {
                    OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
                                getName().c_str());
                    opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
                    // Store the new objective in the problem def'n
                    pdef_->setOptimizationObjective(opt_);
                }
                Open_[0].getComparisonOperator().opt_ = opt_.get();
                Open_[0].getComparisonOperator().heuristics_ = heuristics_;
                Open_[1].getComparisonOperator().opt_ = opt_.get();
                Open_[1].getComparisonOperator().heuristics_ = heuristics_;

                if (!nn_)
                    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<BiDirMotion *>(this));
                nn_->setDistanceFunction([this](const BiDirMotion *a, const BiDirMotion *b)
                                         {
                                             return distanceFunction(a, b);
                                         });

                if (nearestK_ && !nn_->reportsSortedResults())
                {
                    OMPL_WARN("%s: NearestNeighbors datastructure does not return sorted solutions. Nearest K strategy "
                              "disabled.",
                              getName().c_str());
                    nearestK_ = false;
                }
            }
            else
            {
                OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
                setup_ = false;
            }
        }

        void BFMT::freeMemory()
        {
            if (nn_)
            {
                BiDirMotionPtrs motions;
                nn_->list(motions);
                for (auto &motion : motions)
                {
                    si_->freeState(motion->getState());
                    delete motion;
                }
            }
        }

        void BFMT::clear()
        {
            Planner::clear();
            sampler_.reset();
            freeMemory();
            if (nn_)
                nn_->clear();
            Open_[FWD].clear();
            Open_[REV].clear();
            Open_elements[FWD].clear();
            Open_elements[REV].clear();
            neighborhoods_.clear();
            collisionChecks_ = 0;
        }

        void BFMT::getPlannerData(base::PlannerData &data) const
        {
            base::Planner::getPlannerData(data);
            BiDirMotionPtrs motions;
            nn_->list(motions);

            int numStartNodes = 0;
            int numGoalNodes = 0;
            int numEdges = 0;
            int numFwdEdges = 0;
            int numRevEdges = 0;

            int fwd_tree_tag = 1;
            int rev_tree_tag = 2;

            for (auto motion : motions)
            {
                bool inFwdTree = (motion->currentSet_[FWD] != BiDirMotion::SET_UNVISITED);

                // For samples added to the fwd tree, add incoming edges (from fwd tree parent)
                if (inFwdTree)
                {
                    if (motion->parent_[FWD] == nullptr)
                    {
                        // Motion is a forward tree root node
                        ++numStartNodes;
                    }
                    else
                    {
                        bool success =
                            data.addEdge(base::PlannerDataVertex(motion->parent_[FWD]->getState(), fwd_tree_tag),
                                         base::PlannerDataVertex(motion->getState(), fwd_tree_tag));
                        if (success)
                        {
                            ++numFwdEdges;
                            ++numEdges;
                        }
                    }
                }
            }

            // The edges in the goal tree are reversed so that they are in the same direction as start tree
            for (auto motion : motions)
            {
                bool inRevTree = (motion->currentSet_[REV] != BiDirMotion::SET_UNVISITED);

                // For samples added to a tree, add incoming edges (from fwd tree parent)
                if (inRevTree)
                {
                    if (motion->parent_[REV] == nullptr)
                    {
                        // Motion is a reverse tree root node
                        ++numGoalNodes;
                    }
                    else
                    {
                        bool success =
                            data.addEdge(base::PlannerDataVertex(motion->getState(), rev_tree_tag),
                                         base::PlannerDataVertex(motion->parent_[REV]->getState(), rev_tree_tag));
                        if (success)
                        {
                            ++numRevEdges;
                            ++numEdges;
                        }
                    }
                }
            }
        }

        void BFMT::saveNeighborhood(BiDirMotion *m)
        {
            // Check if neighborhood has already been saved
            if (neighborhoods_.find(m) == neighborhoods_.end())
            {
                BiDirMotionPtrs neighborhood;
                if (nearestK_)
                    nn_->nearestK(m, NNk_, neighborhood);
                else
                    nn_->nearestR(m, NNr_, neighborhood);

                if (!neighborhood.empty())
                {
                    // Save the neighborhood but skip the first element (m)
                    neighborhoods_[m] = std::vector<BiDirMotion *>(neighborhood.size() - 1, nullptr);
                    std::copy(neighborhood.begin() + 1, neighborhood.end(), neighborhoods_[m].begin());
                }
                else
                {
                    // Save an empty neighborhood
                    neighborhoods_[m] = std::vector<BiDirMotion *>(0);
                }
            }
        }

        void BFMT::sampleFree(const std::shared_ptr<NearestNeighbors<BiDirMotion *>> &nn,
                              const base::PlannerTerminationCondition &ptc)
        {
            unsigned int nodeCount = 0;
            unsigned int sampleAttempts = 0;
            auto *motion = new BiDirMotion(si_, &tree_);

            // Sample numSamples_ number of nodes from the free configuration space
            while (nodeCount < numSamples_ && !ptc)
            {
                sampler_->sampleUniform(motion->getState());
                sampleAttempts++;
                if (si_->isValid(motion->getState()))
                {  // collision checking
                    ++nodeCount;
                    nn->add(motion);
                    motion = new BiDirMotion(si_, &tree_);
                }
            }
            si_->freeState(motion->getState());
            delete motion;

            // 95% confidence limit for an upper bound for the true free space volume
            freeSpaceVolume_ =
                boost::math::binomial_distribution<>::find_upper_bound_on_p(sampleAttempts, nodeCount, 0.05) *
                si_->getStateSpace()->getMeasure();
        }

        double BFMT::calculateUnitBallVolume(const unsigned int dimension) const
        {
            if (dimension == 0)
                return 1.0;
            if (dimension == 1)
                return 2.0;
            return 2.0 * boost::math::constants::pi<double>() / dimension * calculateUnitBallVolume(dimension - 2);
        }

        double BFMT::calculateRadius(const unsigned int dimension, const unsigned int n) const
        {
            double a = 1.0 / (double)dimension;
            double unitBallVolume = calculateUnitBallVolume(dimension);

            return radiusMultiplier_ * 2.0 * std::pow(a, a) * std::pow(freeSpaceVolume_ / unitBallVolume, a) *
                   std::pow(log((double)n) / (double)n, a);
        }

        void BFMT::initializeProblem(base::GoalSampleableRegion *&goal_s)
        {
            checkValidity();
            if (!sampler_)
            {
                sampler_ = si_->allocStateSampler();
            }
            goal_s = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
        }

        base::PlannerStatus BFMT::solve(const base::PlannerTerminationCondition &ptc)
        {
            base::GoalSampleableRegion *goal_s;
            initializeProblem(goal_s);
            if (goal_s == nullptr)
            {
                OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
                return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
            }

            useFwdTree();

            // Add start states to Unvisitedfwd and Openfwd
            bool valid_initMotion = false;
            BiDirMotion *initMotion;
            while (const base::State *st = pis_.nextStart())
            {
                initMotion = new BiDirMotion(si_, &tree_);
                si_->copyState(initMotion->getState(), st);

                initMotion->currentSet_[REV] = BiDirMotion::SET_UNVISITED;
                nn_->add(initMotion);  // S <-- {x_init}
                if (si_->isValid(initMotion->getState()))
                {
                    // Take the first valid initial state as the forward tree root
                    Open_elements[FWD][initMotion] = Open_[FWD].insert(initMotion);
                    initMotion->currentSet_[FWD] = BiDirMotion::SET_OPEN;
                    initMotion->cost_[FWD] = opt_->initialCost(initMotion->getState());
                    valid_initMotion = true;
                    heurGoalState_[1] = initMotion->getState();
                }
            }

            if ((initMotion == nullptr) || !valid_initMotion)
            {
                OMPL_ERROR("Start state undefined or invalid.");
                return base::PlannerStatus::INVALID_START;
            }

            // Sample N free states in configuration state_
            sampleFree(nn_, ptc);  // S <-- SAMPLEFREE(N)
            OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                        nn_->size());

            // Calculate the nearest neighbor search radius
            if (nearestK_)
            {
                NNk_ = std::ceil(std::pow(2.0 * radiusMultiplier_, (double)si_->getStateDimension()) *
                                 (boost::math::constants::e<double>() / (double)si_->getStateDimension()) *
                                 log((double)nn_->size()));
                OMPL_DEBUG("Using nearest-neighbors k of %d", NNk_);
            }
            else
            {
                NNr_ = calculateRadius(si_->getStateDimension(), nn_->size());
                OMPL_DEBUG("Using radius of %f", NNr_);
            }

            // Add goal states to Unvisitedrev and Openrev
            bool valid_goalMotion = false;
            BiDirMotion *goalMotion;
            while (const base::State *st = pis_.nextGoal())
            {
                goalMotion = new BiDirMotion(si_, &tree_);
                si_->copyState(goalMotion->getState(), st);

                goalMotion->currentSet_[FWD] = BiDirMotion::SET_UNVISITED;
                nn_->add(goalMotion);  // S <-- {x_goal}
                if (si_->isValid(goalMotion->getState()))
                {
                    // Take the first valid goal state as the reverse tree root
                    Open_elements[REV][goalMotion] = Open_[REV].insert(goalMotion);
                    goalMotion->currentSet_[REV] = BiDirMotion::SET_OPEN;
                    goalMotion->cost_[REV] = opt_->terminalCost(goalMotion->getState());
                    valid_goalMotion = true;
                    heurGoalState_[0] = goalMotion->getState();
                }
            }

            if ((goalMotion == nullptr) || !valid_goalMotion)
            {
                OMPL_ERROR("Goal state undefined or invalid.");
                return base::PlannerStatus::INVALID_GOAL;
            }

            useRevTree();

            // Plan a path
            BiDirMotion *connection_point = nullptr;
            bool earlyFailure = true;

            if (initMotion != nullptr && goalMotion != nullptr)
            {
                earlyFailure = plan(initMotion, goalMotion, connection_point, ptc);
            }
            else
            {
                OMPL_ERROR("Initial/goal state(s) are undefined!");
            }

            if (earlyFailure)
            {
                return base::PlannerStatus(false, false);
            }

            // Save the best path (through z)
            if (!ptc)
            {
                base::Cost fwd_cost, rev_cost, connection_cost;

                // Construct the solution path
                useFwdTree();
                BiDirMotionPtrs path_fwd;
                tracePath(connection_point, path_fwd);
                fwd_cost = connection_point->getCost();

                useRevTree();
                BiDirMotionPtrs path_rev;
                tracePath(connection_point, path_rev);
                rev_cost = connection_point->getCost();

                // ASSUMES FROM THIS POINT THAT z = path_fwd[0] = path_rev[0]
                // Remove the first element, z, in the traced reverse path
                // (the same as the first element in the traced forward path)
                if (path_rev.size() > 1)
                {
                    connection_cost = base::Cost(rev_cost.value() - path_rev[1]->getCost().value());
                    path_rev.erase(path_rev.begin());
                }
                else if (path_fwd.size() > 1)
                {
                    connection_cost = base::Cost(fwd_cost.value() - path_fwd[1]->getCost().value());
                    path_fwd.erase(path_fwd.begin());
                }
                else
                {
                    OMPL_ERROR("Solution path traced incorrectly or otherwise constructed improperly \
                through forward/reverse trees (both paths are one node in length, each).");
                }

                // Adjust costs/parents in reverse tree nodes as cost/direction from forward tree root
                useFwdTree();
                path_rev[0]->setCost(base::Cost(path_fwd[0]->getCost().value() + connection_cost.value()));
                path_rev[0]->setParent(path_fwd[0]);
                for (unsigned int i = 1; i < path_rev.size(); ++i)
                {
                    path_rev[i]->setCost(
                        base::Cost(fwd_cost.value() + (rev_cost.value() - path_rev[i]->getCost().value())));
                    path_rev[i]->setParent(path_rev[i - 1]);
                }

                BiDirMotionPtrs mpath;
                std::reverse(path_rev.begin(), path_rev.end());
                mpath.reserve(path_fwd.size() + path_rev.size());  // preallocate memory
                mpath.insert(mpath.end(), path_rev.begin(), path_rev.end());
                mpath.insert(mpath.end(), path_fwd.begin(), path_fwd.end());

                // Set the solution path
                auto path(std::make_shared<PathGeometric>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                {
                    path->append(mpath[i]->getState());
                }

                static const bool approximate = false;
                static const double cost_difference_from_goal = 0.0;
                pdef_->addSolutionPath(path, approximate, cost_difference_from_goal, getName());

                OMPL_DEBUG("Total path cost: %f\n", fwd_cost.value() + rev_cost.value());
                return base::PlannerStatus(true, false);
            }

            // Planner terminated without accomplishing goal
            return {false, false};
        }

        void BFMT::expandTreeFromNode(BiDirMotion *&z, BiDirMotion *&connection_point)
        {
            // Define Opennew and set it to NULL
            BiDirMotionPtrs Open_new;

            // Define Znear as all unexplored nodes in the neighborhood around z
            BiDirMotionPtrs zNear;
            const BiDirMotionPtrs &zNeighborhood = neighborhoods_[z];

            for (auto i : zNeighborhood)
            {
                if (i->getCurrentSet() == BiDirMotion::SET_UNVISITED)
                {
                    zNear.push_back(i);
                }
            }

            // For each node x in Znear
            for (auto x : zNear)
            {
                if (!precomputeNN_)
                    saveNeighborhood(x);  // nearest neighbors

                // Define Xnear as all frontier nodes in the neighborhood around the unexplored node x
                BiDirMotionPtrs xNear;
                const BiDirMotionPtrs &xNeighborhood = neighborhoods_[x];
                for (auto j : xNeighborhood)
                {
                    if (j->getCurrentSet() == BiDirMotion::SET_OPEN)
                    {
                        xNear.push_back(j);
                    }
                }
                // Find the node in Xnear with minimum cost-to-come in the current tree
                BiDirMotion *xMin = nullptr;
                double cMin = std::numeric_limits<double>::infinity();
                for (auto &j : xNear)
                {
                    // check if node costs are smaller than minimum
                    double cNew = j->getCost().value() + distanceFunction(j, x);

                    if (cNew < cMin)
                    {
                        xMin = j;
                        cMin = cNew;
                    }
                }

                // xMin was found
                if (xMin != nullptr)
                {
                    bool collision_free = false;
                    if (cacheCC_)
                    {
                        if (!xMin->alreadyCC(x))
                        {
                            collision_free = si_->checkMotion(xMin->getState(), x->getState());
                            ++collisionChecks_;
                            // Due to FMT3* design, it is only necessary to save unsuccesful
                            // connection attemps because of collision
                            if (!collision_free)
                                xMin->addCC(x);
                        }
                    }
                    else
                    {
                        ++collisionChecks_;
                        collision_free = si_->checkMotion(xMin->getState(), x->getState());
                    }

                    if (collision_free)
                    {  // motion between yMin and x is obstacle free
                        // add edge from xMin to x
                        x->setParent(xMin);
                        x->setCost(base::Cost(cMin));
                        xMin->getChildren().push_back(x);

                        if (heuristics_)
                            x->setHeuristicCost(opt_->motionCostHeuristic(x->getState(), heurGoalState_[tree_]));

                        // check if new node x is in the other tree; if so, save result
                        if (x->getOtherSet() != BiDirMotion::SET_UNVISITED)
                        {
                            if (connection_point == nullptr)
                            {
                                connection_point = x;
                                if (termination_ == FEASIBILITY)
                                {
                                    break;
                                }
                            }
                            else
                            {
                                if ((connection_point->cost_[FWD].value() + connection_point->cost_[REV].value()) >
                                    (x->cost_[FWD].value() + x->cost_[REV].value()))
                                {
                                    connection_point = x;
                                }
                            }
                        }

                        Open_new.push_back(x);                      // add x to Open_new
                        x->setCurrentSet(BiDirMotion::SET_CLOSED);  // remove x from Unvisited
                    }
                }
            }  // End "for x in Znear"

            // Remove motion z from binary heap and map
            BiDirMotionBinHeap::Element *zElement = Open_elements[tree_][z];
            Open_[tree_].remove(zElement);
            Open_elements[tree_].erase(z);
            z->setCurrentSet(BiDirMotion::SET_CLOSED);

            // add nodes in Open_new to Open
            for (auto &i : Open_new)
            {
                if(Open_elements[tree_][i] == nullptr)
                {
                    Open_elements[tree_][i] = Open_[tree_].insert(i);
                    i->setCurrentSet(BiDirMotion::SET_OPEN);
                }
            }
        }

        bool BFMT::plan(BiDirMotion *x_init, BiDirMotion *x_goal, BiDirMotion *&connection_point,
                        const base::PlannerTerminationCondition &ptc)
        {
            // If pre-computation, find neighborhoods for all N sample nodes plus initial
            // and goal state(s).  Otherwise compute the neighborhoods of the initial and
            // goal states separately and compute the others as needed.
            BiDirMotionPtrs sampleNodes;
            nn_->list(sampleNodes);
            /// \todo This precomputation is useful only if the same planner is used many times.
            /// otherwise is probably a waste of time. Do a real precomputation before calling solve().
            if (precomputeNN_)
            {
                for (auto &sampleNode : sampleNodes)
                {
                    saveNeighborhood(sampleNode);  // nearest neighbors
                }
            }
            else
            {
                saveNeighborhood(x_init);  // nearest neighbors
                saveNeighborhood(x_goal);  // nearest neighbors
            }

            // Copy nodes in the sample set to Unvisitedfwd.  Overwrite the label of the initial
            // node with set Open for the forward tree, since it starts in set Openfwd.
            useFwdTree();
            for (auto &sampleNode : sampleNodes)
            {
                sampleNode->setCurrentSet(BiDirMotion::SET_UNVISITED);
            }
            x_init->setCurrentSet(BiDirMotion::SET_OPEN);

            // Copy nodes in the sample set to Unvisitedrev.  Overwrite the label of the goal
            // node with set Open for the reverse tree, since it starts in set Openrev.
            useRevTree();
            for (auto &sampleNode : sampleNodes)
            {
                sampleNode->setCurrentSet(BiDirMotion::SET_UNVISITED);
            }
            x_goal->setCurrentSet(BiDirMotion::SET_OPEN);

            // Expand the trees until reaching the termination condition
            bool earlyFailure = false;
            bool success = false;

            useFwdTree();
            BiDirMotion *z = x_init;

            while (!success)
            {
                expandTreeFromNode(z, connection_point);

                // Check if the algorithm should terminate.  Possibly redefines connection_point.
                if (termination(z, connection_point, ptc))
                    success = true;
                else
                {
                    if (Open_[tree_].empty())  // If this heap is empty...
                    {
                        if (!extendedFMT_)  // ... eFMT not enabled...
                        {
                            if (Open_[(tree_ + 1) % 2].empty())  // ... and this one, failure.
                            {
                                OMPL_INFORM("Both Open are empty before path was found --> no feasible path exists");
                                earlyFailure = true;
                                return earlyFailure;
                            }
                        }
                        else  // However, if eFMT is enabled, run it.
                            insertNewSampleInOpen(ptc);
                    }

                    // This function will be always reached with at least one state in one heap.
                    // However, if ptc terminates, we should skip this.
                    if (!ptc)
                        chooseTreeAndExpansionNode(z);
                    else
                        return true;
                }
            }
            earlyFailure = false;
            return earlyFailure;
        }

        void BFMT::insertNewSampleInOpen(const base::PlannerTerminationCondition &ptc)
        {
            // Sample and connect samples to tree only if there is
            // a possibility to connect to unvisited nodes.
            std::vector<BiDirMotion *> nbh;
            std::vector<base::Cost> costs;
            std::vector<base::Cost> incCosts;
            std::vector<std::size_t> sortedCostIndices;

            // our functor for sorting nearest neighbors
            CostIndexCompare compareFn(costs, *opt_);

            auto *m = new BiDirMotion(si_, &tree_);
            while (!ptc && Open_[tree_].empty())  //&& oneSample)
            {
                // Get new sample and check whether it is valid.
                sampler_->sampleUniform(m->getState());
                if (!si_->isValid(m->getState()))
                    continue;

                // Get neighbours of the new sample.
                std::vector<BiDirMotion *> yNear;
                if (nearestK_)
                    nn_->nearestK(m, NNk_, nbh);
                else
                    nn_->nearestR(m, NNr_, nbh);

                yNear.reserve(nbh.size());
                for (auto &j : nbh)
                {
                    if (j->getCurrentSet() == BiDirMotion::SET_CLOSED)
                    {
                        if (nearestK_)
                        {
                            // Only include neighbors that are mutually k-nearest
                            // Relies on NN datastructure returning k-nearest in sorted order
                            const base::Cost connCost = opt_->motionCost(j->getState(), m->getState());
                            const base::Cost worstCost =
                                opt_->motionCost(neighborhoods_[j].back()->getState(), j->getState());

                            if (opt_->isCostBetterThan(worstCost, connCost))
                                continue;
                            yNear.push_back(j);
                        }
                        else
                            yNear.push_back(j);
                    }
                }

                // Sample again if the new sample does not connect to the tree.
                if (yNear.empty())
                    continue;

                // cache for distance computations
                //
                // Our cost caches only increase in size, so they're only
                // resized if they can't fit the current neighborhood
                if (costs.size() < yNear.size())
                {
                    costs.resize(yNear.size());
                    incCosts.resize(yNear.size());
                    sortedCostIndices.resize(yNear.size());
                }

                // Finding the nearest neighbor to connect to
                // By default, neighborhood states are sorted by cost, and collision checking
                // is performed in increasing order of cost
                //
                // calculate all costs and distances
                for (std::size_t i = 0; i < yNear.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(yNear[i]->getState(), m->getState());
                    costs[i] = opt_->combineCosts(yNear[i]->getCost(), incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < yNear.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + yNear.size(), compareFn);

                // collision check until a valid motion is found
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin() + yNear.size(); ++i)
                {
                    ++collisionChecks_;
                    if (si_->checkMotion(yNear[*i]->getState(), m->getState()))
                    {
                        const base::Cost incCost = opt_->motionCost(yNear[*i]->getState(), m->getState());
                        m->setParent(yNear[*i]);
                        yNear[*i]->getChildren().push_back(m);
                        m->setCost(opt_->combineCosts(yNear[*i]->getCost(), incCost));
                        m->setHeuristicCost(opt_->motionCostHeuristic(m->getState(), heurGoalState_[tree_]));
                        m->setCurrentSet(BiDirMotion::SET_OPEN);
                        Open_elements[tree_][m] = Open_[tree_].insert(m);

                        nn_->add(m);
                        saveNeighborhood(m);
                        updateNeighborhood(m, nbh);

                        break;
                    }
                }
            }  // While Open_[tree_] empty
        }

        bool BFMT::termination(BiDirMotion *&z, BiDirMotion *&connection_point,
                               const base::PlannerTerminationCondition &ptc)
        {
            bool terminate = false;
            switch (termination_)
            {
                case FEASIBILITY:
                    // Test if a connection point was found during tree expansion
                    return (connection_point != nullptr || ptc);
                    break;

                case OPTIMALITY:
                    // Test if z is in SET_CLOSED (interior) of other tree
                    if (ptc)
                        terminate = true;
                    else if (z->getOtherSet() == BiDirMotion::SET_CLOSED)
                        terminate = true;

                    break;
            };
            return terminate;
        }

        // Choose exploration tree and node z to expand
        void BFMT::chooseTreeAndExpansionNode(BiDirMotion *&z)
        {
            switch (exploration_)
            {
                case SWAP_EVERY_TIME:
                    if (Open_[(tree_ + 1) % 2].empty())
                        z = Open_[tree_].top()->data;  // Continue expanding the current tree (not empty by exit
                                                       // condition in plan())
                    else
                    {
                        z = Open_[(tree_ + 1) % 2].top()->data;  // Take top of opposite tree heap as new z
                        swapTrees();                             // Swap to the opposite tree
                    }
                    break;

                case CHOOSE_SMALLEST_Z:
                    BiDirMotion *z1, *z2;
                    if (Open_[(tree_ + 1) % 2].empty())
                        z = Open_[tree_].top()->data;  // Continue expanding the current tree (not empty by exit
                                                       // condition in plan())
                    else if (Open_[tree_].empty())
                    {
                        z = Open_[(tree_ + 1) % 2].top()->data;  // Take top of opposite tree heap as new z
                        swapTrees();                             // Swap to the opposite tree
                    }
                    else
                    {
                        z1 = Open_[tree_].top()->data;
                        z2 = Open_[(tree_ + 1) % 2].top()->data;

                        if (z1->getCost().value() < z2->getOtherCost().value())
                            z = z1;
                        else
                        {
                            z = z2;
                            swapTrees();
                        }
                    }
                    break;
            };
        }

        // Trace a path of nodes along a tree towards the root (forward or reverse)
        void BFMT::tracePath(BiDirMotion *z, BiDirMotionPtrs &path)
        {
            BiDirMotion *solution = z;

            while (solution != nullptr)
            {
                path.push_back(solution);
                solution = solution->getParent();
            }
        }

        void BFMT::swapTrees()
        {
            tree_ = (TreeType)((((int)tree_) + 1) % 2);
        }

        void BFMT::updateNeighborhood(BiDirMotion *m, const std::vector<BiDirMotion *> nbh)
        {
            // Neighborhoods are only updated if the new motion is within bounds (k nearest or within r).
            for (auto i : nbh)
            {
                // If CLOSED, that neighborhood won't be used again.
                // Else, if neighhboorhod already exists, we have to insert the node in
                // the corresponding place of the neighborhood of the neighbor of m.
                if (i->getCurrentSet() == BiDirMotion::SET_CLOSED)
                    continue;

                auto it = neighborhoods_.find(i);
                if (it != neighborhoods_.end())
                {
                    if (it->second.empty())
                        continue;

                    const base::Cost connCost = opt_->motionCost(i->getState(), m->getState());
                    const base::Cost worstCost = opt_->motionCost(it->second.back()->getState(), i->getState());

                    if (opt_->isCostBetterThan(worstCost, connCost))
                        continue;

                    // insert the neighbor in the vector in the correct order
                    std::vector<BiDirMotion *> &nbhToUpdate = it->second;
                    for (std::size_t j = 0; j < nbhToUpdate.size(); ++j)
                    {
                        // If connection to the new state is better than the current neighbor tested, insert.
                        const base::Cost cost = opt_->motionCost(i->getState(), nbhToUpdate[j]->getState());
                        if (opt_->isCostBetterThan(connCost, cost))
                        {
                            nbhToUpdate.insert(nbhToUpdate.begin() + j, m);
                            break;
                        }
                    }
                }
            }
        }
    }  // End "geometric" namespace
}  // End "ompl" namespace
