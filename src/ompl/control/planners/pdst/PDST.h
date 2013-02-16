/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Jonathan Sobieski */



#ifndef OMPL_CONTROL_PLANNERS_PDST_PDST_
#define OMPL_CONTROL_PLANNERS_PDST_PDST_



// Unused headers included in example planner code
//#include <boost/unordered_map.hpp>
//#include <limits>
//#include "ompl/base/ProjectionEvaluator.h"
//#include "ompl/base/spaces/RealVectorStateSpace.h"
//#include "ompl/base/spaces/SO2StateSpace.h"
//#include "ompl/base/State.h"
//#include "ompl/control/planners/PlannerIncludes.h"
//#include "ompl/control/SpaceInformation.h"
//#include "ompl/datastructures/Grid.h"
//#include "ompl/datastructures/PDF.h"
//#include "ompl/tools/config/SelfConfig.h"
//#include "ompl/util/RandomNumbers.h"


#include <list>
#include <queue>
#include <stack>
#include <vector>

#include <cassert>

#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/control/PathControl.h"
#include "ompl/control/PlannerData.h"
#include "ompl/datastructures/BinaryHeap.h"

//#define PDST_DEBUG
#undef PDST_DEBUG



namespace ompl
{

    namespace control
    {

    class PDST : public base::Planner
    {
    protected:

        // Forward declarations
        class Cell;
        class Motion;
        class MotionCompare;

        // Comparator used to order motions in the priority queue
        class MotionCompare
        {
        public:

        // returns true if m1 is lower priority than m2
        bool operator() (Motion * & p1, Motion * & p2) const
        {
            // lowest priority means highest score
            //return p1->computeScore() > p2->computeScore();
            // The OMPL BinaryHeap requires the operator to be the opposite of the C++ STL priority_queue
            return p1->computeScore() < p2->computeScore();
        }
        };

    public:

    PDST(const SpaceInformationPtr &si) : base::Planner(si, "PDST")
        {
        controlSpace_ = si->getControlSpace().get();
        siC_ = si.get();
        sampler_ = si_->allocValidStateSampler();
        controlSampler_ = siC_->allocDirectedControlSampler();

        // TODO: Figure out a reasonable value for this
        maxDistance_ = 0.0;
        projectionDimension_ = 0;
        projectionSpaceBoundaries_ = NULL;

        goalBias_ = 0.05;
        iter_ = 1;

#ifdef PDST_DEBUG
        propagateFailureCount_ = 0;
#endif
        }

        ~PDST(void)
        {
        freeMemory();
        }

        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

        // Clears the planner to run again.
        // TODO: Write clear and write a better comment here.
        void clear(void);

        // TODO: Write freeMemory and write a better comment here.
        void freeMemory(void);

        // optional, if additional setup/configuration is needed, the setup() method can be implemented
        void setup(void);

        // Validates that PDST has been properly initialized, i.e. projection space parameters correctly set.
        virtual void checkValidity(void);

        // Extracts the planner data from the priority queue into data.
        virtual void getPlannerData(base::PlannerData &data) const;


        void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
        {
        projectionEvaluator_ = projectionEvaluator;
        }

        void setProjectionEvaluator(const std::string &name)
        {
        projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
        }

        const base::ProjectionEvaluatorPtr & getProjectionEvaluator(void) const
        {
        return projectionEvaluator_;
        }

        // Functions which must be called when initializing the planner.
        // The projection space is a subset of R^n. The boundaries must be specified as an array of
        // min/max pairs for each dimension's boundaries.
        void setProjectionDimension(const unsigned int dim)
        {
        projectionDimension_ = dim;
        }
        void setProjectionSpaceBoundaries(std::vector<double> *boundaries)
        {
        std::cout << "setProjectionSpaceBoundaries" << std::endl;
        projectionSpaceBoundaries_ = boundaries;
        std::cout << "projectionSpaceBoundaries_->size() = " << projectionSpaceBoundaries_->size() << std::endl;
        }

        void setGoalBias(double goalBias)
        {
        goalBias_ = goalBias;
        }
        double getGoalBias(void) const
        {
        return goalBias_;
        }

        int getIter(void) const
        {
        return iter_;
        }
#ifdef PDST_DEBUG
        int getPropagateFailureCount(void) const
        {
        return propagateFailureCount_;
        }
#endif

    protected:

        // PDST Protected Classes

        // Class representing the tree of motions exploring the state space
        class Motion
        {
        public:


        Motion(
        const ompl::base::State *state,
        Motion *parent,
        ompl::control::Control *control,
        unsigned int controlDuration,
        double priority,
        ompl::base::EuclideanProjection *projection
        )
        /*
          Note: The compiler issues a warning if the initialization list is not in the same order as the
          parameters are declared within the class since it auto-reorders initialization lists to match
          class declaration order.
        */
        :
#ifdef PDST_DEBUG
        motionId_(MOTION_COUNT++),
            propagationFailureCount_(0),
#endif
            priority_(priority),
            parent_(parent),
            state_(state),
            control_(control),
            controlDuration_(controlDuration),
            projection_(projection),
            heapElement_(NULL)
            {
            /*
              propagationFailureCount_ = 0;
              state_ = state;
              parent_ = parent;
              control_ = control;
              controlDuration_ = controlDuration;
              priority_ = priority;
            */
            assert(control_ != NULL || motionId_ == 0);
            }

#ifdef PDST_DEBUG
        // Call once when initializing the Motion class before you use it or whenever you restart PDST
        static void setup(void)
        {
            // TODO: At some point these static variables should perhaps be removed as a performance
            // optimization and to allow multiple PDST objects to work simultaneously.
            // Currently they are only being used for debugging purposes.
            MOTION_COUNT = 0;
            SPLIT_COUNT = 0;
        }

        static unsigned int getSplitCount(void)
        {
            return Motion::SPLIT_COUNT;
        }

        void incrementPropagationFailureCount(void)
        {
            propagationFailureCount_++;
        }
        unsigned int getPropagationFailureCount(void) const
        {
            return propagationFailureCount_;
        }

        unsigned int getMotionId(void) const
        {
            return motionId_;
        }
#endif

        Motion *getParent(void) const
        {
            return parent_;
        }
        void setParent(Motion *parent)
        {
            parent_ = parent;
        }

        // compute score function required
        double computeScore(void) const
        {
            return priority_ / cell_->getDensity();
        }

        // Getter for state
        const ompl::base::State *getState(void) const
        {
            return state_;
        }

        // Getter and setter for control duration
        unsigned int getControlDuration(void) const
        {
            return controlDuration_;
        }
        void setControlDuration(unsigned int controlDuration)
        {
            controlDuration_ = controlDuration;
        }

        // Getter and setter for priority
        double getPriority(void) const
        {
            return priority_;
        }
        void setPriority(double priority)
        {
            priority_ = priority;
        }

        // Getter and setter for cell
        Cell *getCell(void) const
        {
            return cell_;
        }
        void setCell(Cell *cell)
        {
            cell_ = cell;
        }

        const ompl::control::Control *getControl(void) const
        {
            return control_;
        }

        ompl::base::EuclideanProjection *getProjection(void) const
        {
            return projection_;
        }
        void setProjection(ompl::base::EuclideanProjection *projection)
        {
            projection_ = projection;
        }

        ompl::BinaryHeap<Motion *, MotionCompare>::Element *getHeapElement(void) const
        {
            return heapElement_;
        }
        void setHeapElement(ompl::BinaryHeap<Motion *, MotionCompare>::Element *heapElement)
        {
            heapElement_ = heapElement;
        }

        // Splits this Motion into two parts; a new Motion is created which becomes this Motion's parent
        // and is the first part of this Motion; this Motion becomes the second part of the origional Motion.
        // Returns a pointer to the first half of this Motion.
        // Split takes a duration which is the number of steps after which
        // this motion should be split into a new motion.
        // Split returns a pointer to the new motion which is the first half of this motion and leaves this
        // motion as the motion.
        // Depending on which paramters are supplied the motion
        // may have its priority penalized, have its cell set and be added to that
        // cell's list of motions, or added to the priority queue.
        Motion *split(
            unsigned int duration,
            bool penalize,
            const SpaceInformation *si,
            const ompl::base::ProjectionEvaluatorPtr projectionEvaluator,
            ompl::base::State *intermediateState = NULL,
            ompl::base::EuclideanProjection *projection = NULL,
            Cell *cell = NULL
            )
        {
#ifdef PDST_DEBUG
            SPLIT_COUNT++;
#endif

            assert(!(duration < 1 || duration >= controlDuration_));
            assert(parent_ != NULL);

            const ompl::base::State *parentState = parent_->getState();
            
            // If intermediateState == NULL then compute the intermediateState and projection, otherwise
            // they were supplied
            if (intermediateState == NULL)
            {
            assert(projection == NULL);

            intermediateState = si->allocState();

            //std::cout << "Before propagateWhileValid" << std::endl;
#ifdef PDST_DEBUG
            unsigned int propagationDuration =
#endif
                si->propagateWhileValid(parentState, control_, duration, intermediateState);
            //std::cout << "After propagateWhileValid" << std::endl;
#ifdef PDST_DEBUG
            assert(propagationDuration == duration);
#endif

            projection = new ompl::base::EuclideanProjection();
            projectionEvaluator->project(intermediateState, *projection);
            }
            else
            {
            assert(projection != NULL);
            }

            // Create the second half of this motion, penalize it, and add it back into the priority queue.
            Motion *firstPartOfThisMotion = new Motion(intermediateState,
                                   parent_,
                                   control_,
                                   duration,
                                   priority_,
                                   projection);

            if (cell != NULL)
            {
            firstPartOfThisMotion->setCell(cell);
            cell->addMotion(firstPartOfThisMotion);
            }
            else
            {
            firstPartOfThisMotion->setCell(cell_);
            cell_->addMotion(firstPartOfThisMotion);
            }

            // Set this motion's parent to be the intermediate motion
            parent_ = firstPartOfThisMotion;
            controlDuration_ -= duration;
            if (penalize)
            {
            priority_ = 2 * priority_ + 1;
            }
            
            return firstPartOfThisMotion;
        }

        // Function used for debugging purposes to print this motion
        void printMotion(const ompl::base::SpaceInformation *si, std::ostream &out=std::cout) const
        {
#ifdef PDST_DEBUG
            std::cout << "motionId_ = " << motionId_ << std::endl;
            std::cout << "parent->motionId_ = " << ((parent_ != NULL) ? parent_->getMotionId() : 0) << std::endl;
#endif
            assert(si != NULL);
            si->printState(state_, out);
            std::cout << "About to call printControl" << std::endl;
            assert(control_ != NULL || motionId_ == 0);
            if (control_ != NULL)
            {
            ((ompl::control::SpaceInformation *)si)->printControl(control_, out);
            }
            std::cout << "controlDuration_ = " << controlDuration_ << std::endl;
            std::cout << "priority_ = " << priority_ << std::endl;
#ifdef PDST_DEBUG
            std::cout << "propagationFailureCount_ = " << propagationFailureCount_ << std::endl;
#endif
        }
        
        protected:

#ifdef PDST_DEBUG
        // Global unique identifier for instances of a Motion
        static unsigned int MOTION_COUNT;

        // Global count of the number of times split has been called.
        static unsigned int SPLIT_COUNT;

        // Unique identifier for this motion
        const unsigned int motionId_;

        unsigned int propagationFailureCount_;
#endif

        // Priority for selecting this path to extend from in the future
        double priority_;

        // pointer to the cell that contains this path
        Cell *cell_;

        // Parent motion from which this one started
        Motion *parent_;

        // The state achieved by this motion
        const ompl::base::State *state_;

        // The control that was applied to arrive at this state from the parent
        ompl::control::Control *control_;

        // The duration that the control was applied to arrive at this state from the parent
        unsigned int controlDuration_;

        // Cache of the projection for this Motion
        ompl::base::EuclideanProjection *projection_;

        // Handle to the element of the priority queue for this Motion
        ompl::BinaryHeap<Motion *, MotionCompare>::Element *heapElement_;
        };

        // Cell Binary Space Partition
        class Cell {
        public:

        Cell(double density, std::vector<double> *boundaries, unsigned int splitDimension = 0)
        {
            motions_ = new std::list<Motion *>();
            density_ = density;
            boundaries_ = boundaries;
            splitDimension_ = splitDimension;
            left_ = NULL;
            right_ = NULL;
        }

        ~Cell()
        {
            delete motions_;

            if (boundaries_ != NULL)
            {
            delete boundaries_;
            }

            if (left_ != NULL)
            {
            delete left_;
            }

            if (right_ != NULL)
            {
            delete right_;
            }
        }

        // Debugging function used to print cell information
        void printCell(unsigned int dimension) const
        {
            std::cout << "density_ = " << density_ << std::endl;
            std::cout << "splitDimension_ = " << splitDimension_ << std::endl;
            std::cout << "splitValue_ = " << splitValue_ << std::endl;
            std::cout << "left_ = " << left_ << std::endl;
            std::cout << "right_ = " << right_ << std::endl;

            if (boundaries_ != NULL)
            {
            std::cout << "[";
            for (unsigned int dimIdx = 0; dimIdx < dimension; dimIdx++)
            {
                std::cout << "(" << boundaries_->at(2 * dimIdx) << "," << boundaries_->at(2 * dimIdx + 1) << ")";
            }
            std::cout << "]" << std::endl;
            }
            else
            {
            std::cout << "boundaries_ = NULL" << std::endl;
            }

            // could add some debugging logic here to print the list of motions
        }

        // Subdivides this cell
        void subdivide(unsigned int spaceDimension)
        {
            //std::cout << "Subdividing cell:" << std::endl;
            //printCell(spaceDimension);

            double childCellDensity = density_ / 2.0;
            unsigned int nextSplitDimension = (splitDimension_ + 1) % spaceDimension;
            splitValue_ = boundaries_->at(splitDimension_ * 2) +
            ((boundaries_->at(splitDimension_ * 2 + 1) - boundaries_->at(splitDimension_ * 2)) / 2.0);

            assert(!(splitValue_ < boundaries_->at(splitDimension_ * 2) ||
                 splitValue_ > boundaries_->at(splitDimension_ * 2 + 1)));

            std::vector<double> *rightBoundaries = new std::vector<double>(*boundaries_);

            (*boundaries_)[splitDimension_ * 2 + 1] = splitValue_;            
            (*rightBoundaries)[splitDimension_ * 2] = splitValue_;

            left_ = new Cell(childCellDensity, boundaries_, nextSplitDimension);
            right_ = new Cell(childCellDensity, rightBoundaries, nextSplitDimension);
            
            boundaries_ = NULL;

            //std::cout << "Finished subdivide, splitValue_ = " << splitValue_ << std::endl;
        }

        // Locates the cell that this motion begins in
        Cell *stab(const ompl::base::EuclideanProjection *projection)
        {
#ifdef PDST_DEBUG
            //printCell(projection->size());

            //std::cout << "Stabbing for projection:" << std::endl;
            /*
              for (unsigned int projIdx = 0; projIdx < projection->size(); projIdx++)
              {
              std::cout << ((projIdx > 0) ? "," : "") << (*projection)[projIdx];
              }
              std::cout << std::endl;
            */
#endif

            Cell *containingCell = this;
            while (containingCell->left_ != NULL)
            {
            if ((*projection)[containingCell->splitDimension_] <= containingCell->splitValue_)
            {
                containingCell = containingCell->left_;
            }
            else
            {
                containingCell = containingCell->right_;
            }
            }

#ifdef PDST_DEBUG
            //std::cout << "Return from cell.stab" << std::endl;
            //std::cout << "containingCell:" << std::endl;
#endif
            //containingCell->printCell(projection->size());

            return containingCell;
        }

        // Getter for density
        double getDensity() const
        {
            return density_;
        }

        std::list<Motion *> *getMotions() const
        {
            return motions_;
        }

        void reinitializePaths()
        {
            motions_ = new std::list<Motion *>();
        }

        void addMotion(Motion *motion)
        {
            motions_->push_back(motion);
        }

        protected:
        double density_;

        unsigned int splitDimension_;

        double splitValue_;

        Cell *left_;
        
        Cell *right_;

        std::vector<double> *boundaries_;
        
        std::list<Motion *> *motions_;
        };



        // PDST protected member functions

        // Inserts the motion into the appropriate cell
        void insertSampleIntoBsp(Motion *motion, Cell *bsp = NULL);

        Motion *propagateFrom(Motion **startMotion);



        // PDST protected member variables

        // Valid state sampler
        ompl::base::ValidStateSamplerPtr sampler_;

        // Directed control sampler
        DirectedControlSamplerPtr controlSampler_;

        // Control space convenience pointer
        ControlSpace *controlSpace_;
        
        // SpaceInformation convenience pointer
        const SpaceInformation *siC_;

        // Random number generator
        RNG rng_;

        // Maximum distance for sampling random states
        double maxDistance_;

        // Vector holding all of the start states supplied for the problem
        // Each start motion is the root of its own tree of motions.
        std::vector<Motion *> startMotions_;

        // Priority queue of motions
        ompl::BinaryHeap<Motion *, MotionCompare> priorityQueue_;

        // Binary Space Partition
        Cell *bsp_;

        // Projection dimension
        unsigned int projectionDimension_;

        // Projection boundaries
        // This is an array of boundaries for the projection space used in the binary space partition.
        // The user must manually set the projection space boundaries by calling setProjectionDimension()
        // then setProjectionSpaceBoundaries by passing an array of min, max values in each dimension.
        std::vector<double> *projectionSpaceBoundaries_;

        // Projection evaluator for the problem
        ompl::base::ProjectionEvaluatorPtr projectionEvaluator_;

        // Number between 0 and 1 specifying the probability with which the goal should be sampled
        double goalBias_;

        // Objected used to sample the goal
        ompl::base::GoalSampleableRegion *goalSampler_;

        // Iteration number and priority of the next Motion that will be generated
        int iter_;

        // Closest motion to the goal
        // TODO: Ask Mark about wheter subsequent calls to solve after an exact solution has
        // been found should result in the planner running. It appears that for a lot of the
        // planners they might.
        // TODO: Consider changing this variable name to lastGoalMotion_ in order to be more
        // conforming to the style of the other planners.
        Motion *closestMotionToGoal_;

#ifdef PDST_DEBUG
        int propagateFailureCount_;
#endif
    };
    }
}

#endif
