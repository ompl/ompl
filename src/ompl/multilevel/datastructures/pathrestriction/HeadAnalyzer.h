/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#pragma once
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>

namespace ompl
{
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::Head */
        OMPL_CLASS_FORWARD(Head);
        /// @endcond
        class HeadAnalyzer
        {
        public:
            /** \brief Simple debugger for the Head class to write information
             * continuously onto the terminal */
            HeadAnalyzer(HeadPtr &head)
            {
                head_ = head;
            }
            using OccurenceMap = std::map<std::string, int>;

            void operator()(std::string s)
            {
                if (enabled_)
                {
                    auto entry = map_.find(s);
                    if (entry == map_.end())
                    {
                        map_[s] = 1;
                    }
                    else
                    {
                        map_[s] = map_[s] + 1;
                    }
                    samples_++;
                }
            }

            void disable()
            {
                enabled_ = false;
            }

            void clear()
            {
                map_.clear();
            }
            void print()
            {
                if (enabled_)
                {
                    OccurenceMap::iterator itr;
                    std::cout << std::string(80, '-') << std::endl;
                    std::cout << "HeadAnalyzer (" << samples_ << " samples, location " << head_->getLocationOnBasePath()
                              << ")" << std::endl;
                    for (itr = map_.begin(); itr != map_.end(); ++itr)
                    {
                        std::cout << " > " << itr->first << ": " << itr->second << std::endl;
                    }
                    std::cout << std::string(80, '-') << std::endl;
                }
            }

        private:
            OccurenceMap map_;
            int samples_{0};
            HeadPtr head_;

            bool enabled_{true};
        };
    }
}
