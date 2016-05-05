/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_IDGENERATOR_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_IDGENERATOR_

// I am member class of the BITstar class, so I need to include it's definition to be aware of the class BITstar. It has
// a forward declaration to me.
#include "ompl/geometric/planners/bitstar/BITstar.h"

#include <thread>
#include <mutex>
// For boost::scoped_ptr
#include <boost/scoped_ptr.hpp>

namespace ompl
{
    namespace geometric
    {
        /** @anchor IdGenerator
        @par Short description
        A class to generate unique IDs for the \ref gVertex "Vertex" class.
        */

        /** \brief An ID generator class for vertex IDs.*/
        class BITstar::IdGenerator
        {
        public:
            IdGenerator() : nextId_(0u)
            {
            }

            /** \brief Generator a new id and increment the global/static counter of IDs. */
            BITstar::VertexId getNewId()
            {
                // Create a scoped mutex copy of idMutex that unlocks when it goes out of scope:
                std::lock_guard<std::mutex> lockGuard(idMutex_);

                // Return the next id, purposefully post-decrementing:
                return nextId_++;
            }

        private:
            // Variables:
            // The next ID to be returned:
            BITstar::VertexId nextId_;
            // The mutex
            std::mutex idMutex_;
        };
    }  // geometric
}  // ompl

// An anonymous namespace to hide the instance:
namespace
{
    // Global variables:
    // The initialization flag stating that the ID generator has been created:
    static std::once_flag g_IdInited;
    // A pointer to the actual ID generator
    static boost::scoped_ptr<ompl::geometric::BITstar::IdGenerator> g_IdGenerator;

    // A function to initialize the ID generator pointer:
    void initIdGenerator()
    {
        g_IdGenerator.reset(new ompl::geometric::BITstar::IdGenerator());
    }

    // A function to get the current ID generator:
    ompl::geometric::BITstar::IdGenerator &getIdGenerator()
    {
        std::call_once(g_IdInited, &initIdGenerator);
        return *g_IdGenerator;
    }
}
#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_IDGENERATOR_
