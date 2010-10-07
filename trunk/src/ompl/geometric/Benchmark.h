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

#ifndef OMPL_GEOMETRIC_BENCHMARK_
#define OMPL_GEOMETRIC_BENCHMARK_

#include "ompl/geometric/SimpleSetup.h"

namespace ompl
{
    namespace geometric
    {
		
	/** \brief Benchmark a set of planners on a problem instance */
	class Benchmark
	{
	public:
	    
	    /** \brief Constructor needs the manifold needed for planning. */
	    explicit
	    Benchmark(SimpleSetup &setup) : setup_(setup), msg_("Benchmark")
	    {
	    }
	    
	    virtual ~Benchmark(void)
	    {
	    }

	    /** \brief Get the setup the planners are to be tested on */
	    SimpleSetup& getSetup(void) const
	    {
		return setup_;
	    }
	    
	    /** \brief Set the planner to use. If the planner is not
		set, an attempt is made to use the planner
		allocator. If no planner allocator is available
		either, a default planner is set. */
	    void addPlanner(const base::PlannerPtr &planner)
	    {
		if (planner && planner->getSpaceInformation().get() != setup_.getSpaceInformation().get())
		    throw Exception("Planner instance does not match space information");
		planners_.push_back(planner);
	    }
	    
	    /** \brief Set the planner allocator to use. This is only
		used if no planner has been set. This is optional -- a default
		planner will be used if no planner is otherwise specified. */
	    void addPlannerAllocator(const base::PlannerAllocator &pa)
	    {
		planners_.push_back(pa(setup_.getSpaceInformation()));
	    }
	    
	    /** \brief Benchmark the added planners on the defined problem. 
		\param maxTime the maximum amount of time a planner is allowed to run 
		\param maxMem the maximum amount of memory a planner is allowed to use
		\param runCount the number of times to run each planner
		\param dropFirst drop the fastest \e dropFirst runs
		\param dropLast drop the slowest \e dropLast runs */
	    virtual void benchmark(double maxTime, double maxMem, unsigned int runCount);
	    
	    virtual void saveResultsToStream(std::ostream &out = std::cout) const;

	    void saveResultsToFile(const char *filename) const;
	    
	protected:

	    typedef std::map<std::string, std::string> RunProperties;

	    /** \brief The data collected after running a planner multiple times */
	    struct Experiment
	    {
		/// The name of the planner
		std::string                name;

		/// Data collected for each run
		std::vector<RunProperties> runs;

		/// Some common properties averaged from the
		/// runs. This is not extra information. It is simply
		/// useful for quick inspection of results.
		RunProperties              avg;
	    };
	    
	    /** \brief The instance of the problem to benchmark */
	    SimpleSetup                  &setup_;

	    /// The set of planners to be tested
	    std::vector<base::PlannerPtr> planners_;

	    /// The collected experimental data; each element of the array (an experiment) corresponds to a planner
	    std::vector<Experiment>       exp_;
    	    
	    
	    /// Interface for console output
	    msg::Interface                msg_;
	    
	};
    }
    
}
#endif
