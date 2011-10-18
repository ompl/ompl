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

#ifndef OMPL_TOOLS_SPACES_STATE_SPACE_COLLECTION_
#define OMPL_TOOLS_SPACES_STATE_SPACE_COLLECTION_

#include "ompl/base/StateSpace.h"
#include "ompl/util/Console.h"
#include <vector>
#include <string>

namespace ompl
{

    /** \brief Manage a set of state spaces such that they share common subspaces.

        This is useful when dealing with multiple compound state spaces that they
        are constructed from the same smaller subspaces. Sharing subspaces is important when using the functionality offered by the
        \ref stateAndSpaceOperators "state space operators". */
    class StateSpaceCollection
    {
    public:

        StateSpaceCollection(void)
        {
            setAutomaticNames("+", "", "");
        }

        ~StateSpaceCollection(void)
        {
        }

        /** \brief Get the name of the collection */
        const std::string& getName(void) const;

        /** \brief Set the name of the collection */
        void setName(const std::string &name);

        /** \brief Make this collection aware of the state space \e space, and all its subspaces */
        void collect(const base::StateSpacePtr &space);

        /** \brief Make this collection aware of a set of state spaces */
        void collect(const std::vector<base::StateSpacePtr> &spaces);

        /** \brief Get the state space whose name is \e name (if the collection is aware of this space; collect() must have been previously called on this space).
            Throws an exception if the space is not found. */
        const base::StateSpacePtr& getSpace(const std::string &name) const;

        /** \brief Check if a particular space is in the collection */
        bool haveSpace(const std::string &name) const;

        /** \brief Check if a particular space is in the collection */
        bool haveSpace(const base::StateSpacePtr &space) const;

        /** \brief Combine the spaces in \e components into one
            compound space (ompl::base::CompoundStateSpace). The weight of
            each component is 1. The generated space is automatically
            collected. If this space has been previously collected by
            this collection, that space is returned instead of
            creating a new instance. */
        const base::StateSpacePtr& combine(const std::vector<base::StateSpacePtr> &components);

        /** \brief Combine the spaces in \e components into one
            compound space (ompl::base::CompoundStateSpace). Only
            components whose bit is set in \e mask are
            considered. This is a convenience function implemented on
            top of the other versions of collect(). The weight of each
            considered component is 1. The generated space is
            automatically collected. If this space has been previously
            collected by this collection, that space is returned
            instead of creating a new instance. */
        const base::StateSpacePtr& combine(const std::vector<base::StateSpacePtr> &components,
                                           const std::vector<bool> &mask);

        /** \brief Combine the spaces in \e components into one
            compound space (ompl::base::CompoundStateSpace). Only
            components whose bit is set in \e mask are
            considered. This is a convenience function implemented on
            top of the other versions of collect(). The weights of the
            components are specified in \e weights. The generated
            space is automatically collected. If this space has been
            previously collected by this collection, that space is
            returned instead of creating a new instance. */
        const base::StateSpacePtr& combine(const std::vector<base::StateSpacePtr> &components,
                                           const std::vector<bool> &mask, const std::vector<double> &weights);

        /** \brief Combine the spaces in \e components into one
            compound space (ompl::base::CompoundStateSpace). The
            weights of the components are in \e weights. The generated
            space is automatically collected. If this space has been
            previously collected by this collection, that space is
            returned instead of creating a new instance. */
        const base::StateSpacePtr& combine(const std::vector<base::StateSpacePtr> &components,
                                           const std::vector<double> &weights);

        /** \brief Given the state spaces \e components, generate all
            combinations of compound state spaces. Weights of
            subspaces are always set to 1. All generated spaces are
            collected. */
        std::vector<base::StateSpacePtr> allCombinations(const std::vector<base::StateSpacePtr> &components);

        /** \brief Given the state spaces \e components, generate all
            combinations of compound state spaces. Weights of
            subspaces are always specified by \e weights. All
            generated spaces are collected. */
        std::vector<base::StateSpacePtr> allCombinations(const std::vector<base::StateSpacePtr> &components, const std::vector<double> &weights);

        /** \brief By default, generated spaces are named by joining
            the names of the component spaces by \e join, adding \e
            prefix as prefix and \e suffix as suffix. A further prefix
            is set of the collection is named.

            \note Example generated name: [collection name]:[\e prefix][name of component 1][\e join][name of component 2][\e join][name of component 3][\e suffix]*/
        void setAutomaticNames(const std::string &join, const std::string &prefix, const std::string &suffix);

    private:

        std::string                      name_;
        std::vector<base::StateSpacePtr> spaces_;
        std::string                      join_;
        std::string                      prefix_;
        std::string                      suffix_;

        msg::Interface                   msg_;
    };
}

#endif
