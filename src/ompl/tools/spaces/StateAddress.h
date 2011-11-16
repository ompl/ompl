/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef OMPL_TOOLS_SPACES_STATE_ADDRESS_
#define OMPL_TOOLS_SPACES_STATE_ADDRESS_

#include "ompl/base/StateSpace.h"
#include "ompl/util/Console.h"
#include <vector>
#include <string>
#include <map>

namespace ompl
{

    /** \brief Tool that provides fast access to elements of a state by name, if they are representable as doubles */
    class StateAddress
    {
    public:

        /** \brief Representation of the address of a value in a state */
        struct Location
        {
            /** \brief In a complex state space there may be multiple
                compound state spaces that make up an even larger
                compound space.  This array indicates the sequence of
                indices of the subspaces that need to be followed to
                get to the component of the state that is of interest. */
            std::vector<std::size_t> chain;

            /** \brief The space that is reached if the chain above is followed on the state space */
            const base::StateSpace  *space;

            /** \brief The index of the value to be accessed, within the space above */
            std::size_t              index;
        };

        /** \brief Compute the mapping between names and addresses for \e space. Calls setStateSpace() */
        StateAddress(const base::StateSpacePtr &space);

        /** \brief Empty constructor */
        StateAddress(void);

        ~StateAddress(void);

        /** \brief Change the space that addresses are computed
            for. Names of state spaces and of individual dimensions
            (e.g., ompl::base::RealVectorStateSpace) are considered. */
        void setStateSpace(const base::StateSpacePtr &space);

        /** \brief The space for which the addresses are maintained */
        const base::StateSpacePtr& getStateSpace(void) const
        {
            return space_;
        }

        /** \brief Get the value of \e state at address \e loc */
        double* getValueAddressAtLocation(const Location &loc, base::State *state) const;

        /** \brief Get the value of \e state at address \e loc */
        const double* getValueAddressAtLocation(const Location &loc, const base::State *state) const;

        /** \brief Get the value of \e state at the component named \e name */
        double* getValueAddressAtName(const std::string &name, base::State *state) const;

        /** \brief Get the value of \e state at the component named \e name */
        const double* getValueAddressAtName(const std::string &name, const base::State *state) const;

    private:

        void storeLocation(Location &loc, const base::StateSpace *s);

        base::StateSpacePtr             space_;
        std::map<std::string, Location> locations_;

        msg::Interface                  msg_;
    };
}

#endif
