/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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

// Authors: Marlin Strub

#ifndef OMPL_GEOMETRIC_PLANNERS_EITSTAR_STOPWATCH_STOPWATCH_
#define OMPL_GEOMETRIC_PLANNERS_EITSTAR_STOPWATCH_STOPWATCH_

#include <chrono>
#include <map>
#include <string>

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            namespace timing
            {
                template <typename T>
                class Stopwatch
                {
                public:
                    explicit Stopwatch(const std::string &name);
                    ~Stopwatch() = default;

                    void start(size_t instance = 0u);
                    void stop(size_t instance = 0u);  // Implementation is in timetable.h.
                    inline bool isTiming(size_t instance = 0u);

                private:
                    std::map<size_t, std::chrono::steady_clock::time_point> starts_;
                    const std::string name_;
                };

                template <typename T>
                Stopwatch<T>::Stopwatch(const std::string &name) : starts_(), name_(name)
                {
                }

                template <typename T>
                void Stopwatch<T>::start(size_t instance)
                {
                    if (starts_.count(instance))
                    {
                        auto msg = std::string("Stopwatch '") + name_ + std::string(", ") + std::to_string(instance) +
                                   std::string("' is already timing.");
                        throw std::runtime_error(msg);
                    }
                    starts_.emplace(instance, std::chrono::steady_clock::now());
                }

                template <typename T>
                bool Stopwatch<T>::isTiming(size_t instance)
                {
                    return starts_.count(instance);
                }

            }  // namespace timing

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_EITSTAR_STOPWATCH_STOPWATCH_
