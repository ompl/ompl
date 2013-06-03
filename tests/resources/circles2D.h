/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

#ifndef OMPL_TEST_CIRCLES_2D_
#define OMPL_TEST_CIRCLES_2D_

#include <fstream>
#include <vector>

struct Circles2D
{
    struct Circle
    {
        Circle(double x, double y, double r) : x_(x), y_(y), r_(r), r2_(r*r)
        {
        }

        double x_, y_, r_, r2_;
    };

    struct Query
    {
        double startX_, startY_, goalX_, goalY_;
    };

    Circles2D(void)
    {
        minX_ = minY_ = 0.0;
        maxX_ = maxY_ = 0.0;
    }

    void loadCircles(const std::string &filename)
    {
        //    std::cout << "Loading " << filename << std::endl;
        std::ifstream fin(filename.c_str());
        // ignore first two lines
        char dummy[4096];
        fin.getline(dummy, sizeof(dummy));
        fin.getline(dummy, sizeof(dummy));
        while (true)
        {
            int id;
            double x, y, r;
            fin >> id >> x >> y >> r;
            if (fin.eof() || !fin.good())
                break;
            circles_.push_back(Circle(x,y,r));
            //      std::cout << "Added circle " << id << " at center " << x << ", " << y << " of radius " << r << std::endl;
        }
        fin.close();

        // find a bounding box for the environment
        if (circles_.empty())
        {
            minX_ = minY_ = 0.0;
            maxX_ = maxY_ = 0.0;
        }
        else
        {
            minX_ = minY_ = std::numeric_limits<double>::infinity();
            maxX_ = maxY_ = -std::numeric_limits<double>::infinity();
            for (std::size_t i = 0 ; i < circles_.size() ; ++i)
            {
                if (circles_[i].x_ - circles_[i].r_ < minX_)
                    minX_ = circles_[i].x_ - circles_[i].r_;
                if (circles_[i].y_ - circles_[i].r_ < minY_)
                    minY_ = circles_[i].y_ - circles_[i].r_;
                if (circles_[i].x_ + circles_[i].r_ > maxX_)
                    maxX_ = circles_[i].x_ + circles_[i].r_;
                if (circles_[i].y_ + circles_[i].r_ > maxY_)
                    maxY_ = circles_[i].y_ + circles_[i].r_;
            }
        }
        //    std::cout << "Bounding box is [" << minX_ << ", " << minY_ << "] x [" << maxX_ << ", " << maxY_ << "]" << std::endl;
    }

    void loadQueries(const std::string &filename)
    {
        std::ifstream fin(filename.c_str());
        while (fin.good() && !fin.eof())
        {
            std::string dummy;
            Query q;
            fin >> dummy >> dummy >> q.startX_ >> q.startY_;
            if (fin.eof())
                break;
            fin >> dummy >> dummy >> q.goalX_ >> q.goalY_;
            queries_.push_back(q);
        }
    }

    const Query& getQuery(std::size_t index) const
    {
        return queries_[index];
    }

    std::size_t getQueryCount(void) const
    {
        return queries_.size();
    }

    bool noOverlap(double x, double y) const
    {
        for (std::size_t i = 0 ; i < circles_.size() ; ++i)
        {
            double dx = circles_[i].x_ - x;
            double dy = circles_[i].y_ - y;
            if (dx * dx + dy * dy < circles_[i].r2_)
                return false;
        }
        return true;
    }

    std::vector<Circle> circles_;
    std::vector<Query> queries_;
    double minX_;
    double maxX_;
    double minY_;
    double maxY_;
};

#endif
