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

#ifndef OMPL_GEOMETRIC_PLANNERS_EITSTAR_STOPWATCH_TIMETABLE_
#define OMPL_GEOMETRIC_PLANNERS_EITSTAR_STOPWATCH_TIMETABLE_

#include <stdio.h>
#include <sys/ioctl.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <mutex>
#include <string>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include "ompl/geometric/planners/eitstar/stopwatch/stopwatch.h"

#define EITSTAR_TIMING 1

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            namespace timing
            {
                // TODO(Marlin): The value type should really be an integer. However, the
                // 'max' tag returns the max value that can be safed in the int type instead
                // of the max value of the measurements.
                using AccumulatorSet = boost::accumulators::accumulator_set<
                    double, boost::accumulators::features<boost::accumulators::tag::sum, boost::accumulators::tag::min,
                                                          boost::accumulators::tag::max, boost::accumulators::tag::mean,
                                                          boost::accumulators::tag::median,
                                                          boost::accumulators::tag::lazy_variance>>;

                template <typename T = std::chrono::microseconds>
                class Timetable
                {
                public:
                    friend class Stopwatch<T>;

                    static std::unique_ptr<Stopwatch<T>> createStopwatch(const std::string &name);

                    // Make public how many stopwatches there are.
                    static size_t getNumStopwatches();

                    // Reset the timetable, deleting all stopwatches.
                    static void reset();

                    // Provide access to the timetable info.
                    static size_t getNumMeasurments(const std::string &name);
                    static uint64_t getTotal(const std::string &name);
                    static uint64_t getMin(const std::string &name);
                    static uint64_t getMax(const std::string &name);
                    static uint64_t getMean(const std::string &name);
                    static uint64_t getMedian(const std::string &name);
                    static uint64_t getVariance(const std::string &name);
                    static double getStandardDeviation(const std::string &name);

                    // Print the timetable.
                    static void print(std::ostream &out);

                    // Export the timetable as csv.
                    static void exportCsv(const std::string &filename);

                private:
                    // Implement the singletion pattern.
                    Timetable();
                    ~Timetable() = default;
                    static Timetable<T> &instance();

                    // Add a measurement to an accumulator.
                    void addMeasurement(const std::string &name, const uint64_t duration);

                    // Get units of this timetable as a string.
                    static const std::string units();

                    // Hold all of the stopwatches in a threadsafe map.
                    std::mutex accumulators_mutex_;
                    std::map<std::string, AccumulatorSet> accumulators_;
                };

                template <typename T>
                Timetable<T>::Timetable() : accumulators_mutex_(), accumulators_()
                {
                }

                template <typename T>
                Timetable<T> &Timetable<T>::instance()
                {
                    static Timetable<T> timetable;
                    return timetable;
                }

                template <typename T>
                std::unique_ptr<Stopwatch<T>> Timetable<T>::createStopwatch(const std::string &name)
                {
                    instance().accumulators_.emplace(name, AccumulatorSet());
                    return std::unique_ptr<Stopwatch<T>>(new Stopwatch<T>(name));
                }

                template <typename T>
                size_t Timetable<T>::getNumStopwatches()
                {
                    return instance().accumulators_.size();
                }

                template <typename T>
                void Timetable<T>::reset()
                {
                    instance().accumulators_.clear();
                }

                template <typename T>
                size_t Timetable<T>::getNumMeasurments(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract::count(instance().accumulators_[name]);
                }

                template <typename T>
                uint64_t Timetable<T>::getTotal(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract_result<boost::accumulators::tag::sum>(
                        instance().accumulators_[name]);
                }

                template <typename T>
                uint64_t Timetable<T>::getMin(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract_result<boost::accumulators::tag::min>(
                        instance().accumulators_[name]);
                }

                template <typename T>
                uint64_t Timetable<T>::getMax(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract_result<::boost::accumulators::tag::max>(
                        instance().accumulators_[name]);
                }

                template <typename T>
                uint64_t Timetable<T>::getMean(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract_result<boost::accumulators::tag::mean>(
                        instance().accumulators_[name]);
                }

                template <typename T>
                uint64_t Timetable<T>::getMedian(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract_result<boost::accumulators::tag::median>(
                        instance().accumulators_[name]);
                }

                template <typename T>
                uint64_t Timetable<T>::getVariance(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return boost::accumulators::extract_result<boost::accumulators::tag::variance>(
                        instance().accumulators_[name]);
                }

                template <typename T>
                double Timetable<T>::getStandardDeviation(const std::string &name)
                {
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot find a stopwatch named '") + name + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    return std::sqrt(
                        static_cast<double>(boost::accumulators::extract_result<boost::accumulators::tag::variance>(
                            instance().accumulators_[name])));
                }

                template <typename T>
                void Timetable<T>::addMeasurement(const std::string &name, const uint64_t duration)
                {
                    std::lock_guard<std::mutex> lock(instance().accumulators_mutex_);
                    if (!instance().accumulators_.count(name))
                    {
                        auto msg = std::string("Cannot add measurement to stopwatch named '") + name +
                                   std::string("', as no such stopwatch exists.");
                        throw std::runtime_error(msg);
                    }
                    instance().accumulators_[name](duration);
                }

                template <typename T>
                void Timetable<T>::print(std::ostream &out)
                {
                    // Define the output width.
                    constexpr size_t width = 100;

                    out << std::setw(width) << std::setfill('=') << '\n'
                        << std::setfill(' ') << "\nTimetable "
                        << " [ " << units() << " ]\n"
                        << std::right << std::setw(width / 6.0) << "# Timings" << std::setw(width / 6.0) << "Total Time"
                        << std::setw(width / 3.0) << "( Average +- Std Dev )" << std::setw(width / 3.0)
                        << "[ Min, Max ]" << '\n'
                        << std::setw(width) << std::setfill('=') << '\n'
                        << std::setfill(' ');

                    for (const auto &accumulator : instance().accumulators_)
                    {
                        // Remember the accumulator name.
                        const std::string &name = accumulator.first;
                        out.width(width);
                        out << std::left << name << '\n';
                        out << std::right << std::setw(width / 6.0) << getNumMeasurments(name);
                        if (getNumMeasurments(name) > 0)
                        {
                            out << std::setw(width / 6.0) << instance().getTotal(name);
                            out << std::setw(width / 3.0)
                                << std::string(std::string("( ") + std::to_string(instance().getMean(name)) + " +- " +
                                               std::to_string(static_cast<int>(
                                                   std::round(instance().getStandardDeviation(name)))) +
                                               " )");
                            out << std::setw(width / 3.0)
                                << std::string(std::string("[ ") + std::to_string(instance().getMin(name)) + ", " +
                                               std::to_string(instance().getMax(name)) + " ]");
                        }
                        out << '\n' << std::setw(width) << std::setfill('-') << '\n' << std::setfill(' ');
                    }
                    out << std::left;
                }

                template <typename T>
                void Timetable<T>::exportCsv(const std::string &filename)
                {
                    std::ofstream file(filename);
                    if (!file.is_open())
                    {
                        auto msg = std::string("Could not open stream '") + filename + std::string("'.");
                        throw std::runtime_error(msg);
                    }
                    file << "# measurements in " << units() << '\n';
                    file << "# stopwatch name,number of measurements,total,"
                         << "median,mean,standard deviation,variance,min,max\n";

                    for (const auto &accumulator : instance().accumulators_)
                    {
                        const std::string &name = accumulator.first;
                        file << name << ',' << getNumMeasurments(name) << ',' << instance().getTotal(name) << ','
                             << instance().getMedian(name) << ',' << instance().getMean(name) << ','
                             << instance().getStandardDeviation(name) << ',' << instance().getVariance(name) << ','
                             << instance().getMin(name) << ',' << instance().getMax(name) << '\n';
                    }

                    file.close();
                }

                template <typename T>
                void Stopwatch<T>::stop(size_t instance)
                {
                    if (!starts_.count(instance))
                    {
                        auto msg = std::string("Cannot stop stopwatch '") + name_ + std::string(", ") +
                                   std::to_string(instance) + std::string("', it is currently not timing.");
                        throw std::runtime_error(msg);
                    }
                    Timetable<T>::instance().addMeasurement(
                        name_,
                        std::chrono::duration_cast<T>(std::chrono::steady_clock::now() - starts_.at(instance)).count());
                    starts_.erase(instance);
                }

            }  // namespace timing

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_EITSTAR_STOPWATCH_TIMETABLE_
