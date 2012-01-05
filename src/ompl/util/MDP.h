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

/* Author: Ioan Sucan */

#ifndef OMPL_UTIL_MDP_
#define OMPL_UTIL_MDP_

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "ompl/util/Console.h"

namespace ompl
{

    /** \brief Implementation of algorithms that solve Markov Decision Processes (MDPs) */
    class MDP
    {
    public:

        /** \brief Constructor can optionally take number of states (\e stateCount) and number of actions (\e actionCount) for the MDP */
        MDP(std::size_t stateCount = 0, std::size_t actionCount = 0) :
            stateCount_(stateCount), actionCount_(actionCount), discount_(0.95), err_(0.0), steps_(0)
        {
            act_.resize(stateCount_);
        }

        /** \brief Set the number of states for this MDP. States are assumed to be numbered from 0 to \e stateCount - 1 */
        void setStateCount(std::size_t stateCount)
        {
            stateCount_ = stateCount;
            act_.resize(stateCount_);
        }

        /** \brief Set the number of actions for this MDP. Actions are assumed to be numbered from 0 to \e actionCount - 1 */
        void setActionCount(std::size_t actionCount)
        {
            actionCount_ = actionCount;
        }

        /** \brief Get the number of states for this MDP */
        std::size_t getStateCount(void) const
        {
            return stateCount_;
        }

        /** \brief Get the number of actions for this MDP */
        std::size_t getActionCount(void) const
        {
            return actionCount_;
        }

        /** \brief Get the discount factor for this MDP */
        double getDiscountFactor(void) const
        {
            return discount_;
        }

        /** \brief Set the discount factor for this MDP */
        void setDiscountFactor(double discount)
        {
            discount_ = discount;
        }

        /** \brief Get the policy that was computed last. The result
            is an array of length equal to the number of states.  The
            \e i<sup>th</sup> element of this vector is the action to
            be taken at state \e i. */
        const std::vector<int>& getPolicy(void) const
        {
            return Pi_;
        }

        /** \brief Get the expected rewards that were computed
            last. The result is an array of length equal to the number
            of states.  The \e i<sup>th</sup> element of this vector
            is the expected reward received at state \e i. */
        const std::vector<double>& getValues(void) const
        {
            return V_;
        }

        /** \brief Get the error (L1 norm) of the reward vector obtained in the last computation */
        double getValueError(void) const
        {
            return err_;
        }

        /** \brief Get the number of iterations performed during the last computation */
        double getIterationSteps(void) const
        {
            return steps_;
        }

        /** \brief Record the fact that it is possible to move from
            state \e from to state \e to using action \e act. The
            probability of success is \e prob, and the reward received
            is equal to \e reward. */
        void addPossibleAction(std::size_t from, std::size_t to, std::size_t act,
                               double prob, double reward);
	
	/** \brief Get the probability associated to a particular action */
	double getActionProbability(std::size_t from, std::size_t to, std::size_t act) const;
	
	/** \brief Get the reward associated to a particular action */
	double getActionReward(std::size_t from, std::size_t to, std::size_t act) const;
	
	/** \brief Associate a string \e name to a state \e state. Useful for debug/display purposes */
	void setStateName(std::size_t state, const std::string &name);

	/** \brief Associate a string \e name to an action \e act. Useful for debug/display purposes */
	void setActionName(std::size_t act, const std::string &name);
	
        /** \brief Remove all previously recorded transitions (as if addPossibleAction() was never called) */
        void clear(void);

	/** \brief Clear the previously computed policy */
	void clearPolicy(void);
	
        /** \brief Sanity checks (make sure probabilities sum up to 1) */
        bool check(void) const;

        /** \brief Normalize probabilities so they sum up to 1 */
	void normalize(void);
	
        /** \brief Compute the expected rewards and the corresponding policy using value iteration. Return true if convergence was achieved. */
        bool valueIteration(double tolerance, unsigned int maxSteps);

        /** \brief Extract the most rewarding path between two given
            states, \e start and \e goal in a greedy fashion. The path
            is stored as a sequence of states \e s, which includes \e
            start and \e goal. \e a will contain the actions to be
            taken. It will always be true that \e a.size() = \e
            s.size() - 1. 

	    \note This works only if there are no cycles on the optimal reward path */
        bool extractRewardingPath(std::size_t start, std::size_t goal, std::vector<std::size_t> &s, std::vector<std::size_t> &a) const;

        /** \brief Print the graph structure of this MDP (in graphviz dot format) to a stream */
        void printGraph(std::ostream &out = std::cout) const;

        /** \brief Print information about this MDP to a stream (plain text) */
        void printSettings(std::ostream &out = std::cout) const;

    private:

        /// map from target state to the probability and reward associated to reaching the target state
        typedef std::map<std::size_t, std::pair<double, double> > PossibleDestinations;

        /// map from action to possible destinations
        typedef std::map<std::size_t, PossibleDestinations>       PossibleActions;

        std::size_t                        stateCount_;
        std::size_t                        actionCount_;
        double                             discount_;
        std::vector<PossibleActions>       act_;
	std::map<std::size_t, std::string> stateNames_;
	std::map<std::size_t, std::string> actionNames_;
	
        std::vector<double>                V_;
        std::vector<int>                   Pi_;
        double                             err_;
        double                             steps_;

        msg::Interface                     msg_;
    };

}

#endif
