/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University.
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

/* Author: Bryce Willey */

#ifndef OMPL_TEST_SDF_2D_
#define OMPL_TEST_SDF_2D_

#include <fstream>
#include <iostream>
#include <vector>
#include <limits>
#include <functional>
#include "general2D.h"

struct SignedDistanceField2D : public General2D
{
    SignedDistanceField2D(double minX, double maxX, double minY, double maxY)
        : minX_(minX), maxX_(maxX), minY_(minY), maxY_(maxY)
    {}

    /**
     * Rounds to the closest grid position.
     */
    bool worldToGrid(double x, double y, size_t &ix, size_t &iy) const
    {
        ix = (size_t) round((x - minX_) / df_resolution_);
        iy = (size_t) round((y - minY_) / df_resolution_);
        return ix < num_entries_[0] && iy < num_entries_[1];
    }

    bool noOverlap(double x, double y) const
    {
        size_t ix, iy;
        bool in_bounds = worldToGrid(x, y, ix, iy);
        return in_bounds && signed_distance_[ix][iy] > 0;
    }

    double signedDistance(double x, double y) const
    {
        size_t ix, iy;
        worldToGrid(x, y, ix, iy);
        return signed_distance_[ix][iy] * df_resolution_;
    }

    double obstacleDistanceGradient(double x, double y, Eigen::MatrixXd &grad) const
    {
        size_t ix, iy;
        worldToGrid(x, y, ix, iy);
        size_t lower_x = ix - 1;
        size_t upper_x = ix + 1;
        if (ix == 0)
        {
            lower_x = ix;
        }
        if (ix == num_entries_[0] - 1)
        {
            upper_x = ix;
        }
        grad(0, 0) = (signed_distance_[upper_x][iy] - signed_distance_[lower_x][iy]) / 2.0;

        size_t lower_y = iy - 1;
        size_t upper_y = iy + 1;
        if (iy == 0)
        {
            lower_y = iy;
        }
        if (iy == num_entries_[1] - 1)
        {
            upper_y = iy;
        }
        grad(0, 1) = (signed_distance_[ix][upper_y] - signed_distance_[ix][lower_y]) / 2.0;
        return signed_distance_[ix][iy] * df_resolution_;
    }

    double medialAxisGradient(double x, double y, Eigen::MatrixXd &grad) const
    {
        size_t ix, iy;
        worldToGrid(x, y, ix, iy);
        // take the negative of dist at each location, so we're going up towards 0. 
        // Doesn't matter since we're doing newton's towards 0, but might do extra stuff
        // to increas the value later.
        size_t lower_x = ix - 1;
        size_t upper_x = ix + 1;
        if (ix == 0)
        {
            lower_x = ix;
        }
        if (ix == num_entries_[0] - 1)
        {
            upper_x = ix;
        }
        grad(0, 0) = (dist_from_medial_axis_[upper_x][iy] - dist_from_medial_axis_[lower_x][iy]) / 2.0;

        size_t lower_y = iy - 1;
        size_t upper_y = iy + 1;
        if (iy == 0) 
        {
            lower_y = iy;
        }
        if (iy == num_entries_[1] - 1)
        {
            upper_y = iy;
        }
        grad(0, 1) = (dist_from_medial_axis_[ix][upper_y] - dist_from_medial_axis_[ix][lower_y]) / 2.0;
        return dist_from_medial_axis_[ix][iy] * df_resolution_;
    }

    // TODO: line valid, clearance, obstacle gradient, and axis gradient
    // Look at each grid that the line goes through, and take the min of each of those.
    bool lineNoOverlap(double x1, double y1, double x2, double y2) const
    {
        // Just take points along the line, every df_resolution.
        double delta_x = x2 - x1;
        double delta_y = y2 - y1;
        double length = sqrt(delta_x * delta_x + delta_y + delta_y);
        int pointCount = (int) ceil(length / df_resolution_);
        for (int i = 0; i < pointCount; i++)
        {
            double x = x1 + delta_x * ((i * 1.0) / (pointCount * 1.0));
            double y = y1 + delta_y * ((i * 1.0) / (pointCount * 1.0));
            if (!noOverlap(x, y))
            {
                return false;
            }
        }
        return true;
    }

    double lineSignedDistance(double x1, double y1, double x2, double y2, Eigen::Vector2d & point) const
    {
        // Just take points along the line, every df_resolution.
        double delta_x = x2 - x1;
        double delta_y = y2 - y1;
        double length = sqrt(delta_x * delta_x + delta_y + delta_y);
        int pointCount = (int) ceil(length / df_resolution_);
        double clearance = std::numeric_limits<double>::infinity();
        for (int i = 0; i < pointCount; i++)
        {
            double x = x1 + delta_x * ((i * 1.0) / (pointCount * 1.0));
            double y = y1 + delta_y * ((i * 1.0) / (pointCount * 1.0));
            double dist = signedDistance(x, y);
            if (dist < clearance)
            {
                clearance = dist;
                point[0] = x;
                point[1] = y;
            }
        }
        return clearance;
    }

    /**
     * Common short variable names: (I've kept them to make the algorithm more
     * readable when coming directly from the paper):
     *   f: function to be transformed.
     *   df: the distance transform of f
     *   n: the number of grid cells in this array
     */
    std::vector<double> distance_transform_one_dimension(
            std::function<double (size_t)> f,
            std::function<void (size_t, double)>df,
            std::function<void (size_t, size_t)> set_obj_at_from,
            unsigned int n, bool verbose=false)
    {
        // START: ALGORITHM from Felzenswalb and Huttenlocher - 2004
        // Modified based on or_cdchomp.
        int k = -1; // index of rightmost parabola in lower envelope;
        std::vector<int> v(n); // locations of parabolas in lower envelope.
        std::vector<double> z(n + 1); // locations of boundaries between parabolas.
        z[0] = -1 * std::numeric_limits<double>::infinity();
        z[1] = std::numeric_limits<double>::infinity();
        for (unsigned int q = 0; q < n; q++)
        {
            // Ignore infinite-height parabolas.
            if (f(q) >= std::numeric_limits<double>::max())
            {
                continue;
            }
            
            if (k == -1)
            {
                k = 0;
                v[0] = q;
                continue;
            }

        retry:
            // The intersection of the parabola coming from q and the one coming from v[k].
            double s = ((f(q) + q * q) - (f(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            if (s <= z[k])
            {
                k = k - 1;
                //q--;
                goto retry;
            }
            else
            {
                k = k + 1;
                v[k] = q;
                z[k] = s;
                z[k + 1] = std::numeric_limits<double>::infinity();
            }
        }
        k = 0;
        for (unsigned int q = 0; q < n; q++)
        {
            while (z[k + 1] < q)
            {
                k = k + 1;
            }
            if (verbose)
                std::cout << "f(q): " << f(q) << ", q: " << q << ", k: " << k<< ", v[k]:" << v[k] << ", f(v[k]):" << f(v[k])<< ", z[k]: " << z[k] << ", z[k+ 1]: " << z[k+1] << ", final: " << (q -v[k]) * (q - v[k]) + f(v[k]) << std::endl;
            df(q, (q - v[k]) * (q - v[k]) + f(v[k]));
            set_obj_at_from(q, v[k]);
        }
        return z;
        // END: Felzenswalb and Huttenlocher
    }

    bool almost_equal(double x, double y, int ulp)
    {
        // From C++ reference guide on epsilon.
        // the machine epsilon has to be scaled to the magnitude of the values used
        // and multiplied by the desired precision in ULPs (units in the last place)
        return std::abs(x-y) <= std::numeric_limits<double>::epsilon() * std::abs(x+y) * ulp
        // unless the result is subnormal
               || std::abs(x-y) < std::numeric_limits<double>::min();
    }

    bool setMedialAxisObj(double boundary, double max_idx,
                      std::function<int (size_t)>getObjs,
                      std::function<void (size_t)>setMedial)
    {
        if (boundary < 1.0 || boundary > max_idx - 2)
        {
            return false;
        }
        size_t above = (size_t)floor(boundary + 1);
        size_t below = (size_t)ceil(boundary - 1);

        if (getObjs(above) != getObjs(below))
        {
            setMedial((size_t) round(boundary));
            return true;
        }
        return false;
    }

    /** DEPRECATED */
    bool setMedialAxis_Noisy(double boundary, double max_idx, std::function<double (size_t)> getSD, 
                      std::function<void (size_t)>setMedial)
    {
        //EXP1
        //if (boundary < 1.0 || boundary > max_idx - 2)
        //EXP2
        if (boundary < 2.0 || boundary > max_idx - 3)
        {
            return false;
        }
        // If boundary lies on cell, compare it with the above and below cells.
        if (almost_equal(boundary, round(boundary), 2))
        {
            int bound = (int)boundary;
            int below = (int)bound - 1;
            int above = (int)bound + 1;
            // If the signs of the derivitive of the df is different going to different 
            // sides, then set to 0.
            //EXP1
            double sd_diff_above = getSD(above + 1) - getSD(above);
            double sd_diff_below = getSD(below) - getSD(below - 1);
            //EXP2
            //double sd_diff_above = getSD(above) - getSD(bound);
            //double sd_diff_below = getSD(bound) - getSD(below);
            if ((sd_diff_above > 0) ^ (sd_diff_below > 0) &&
                (sd_diff_above >= 0) ^ (sd_diff_below >= 0))
            {
                setMedial(bound);
                return true;
            }
        }
        else // if boundary is between two cells, get above/below, compare with above +1 etc.
        {
            int below = (int)floor(boundary);
            int above = (int)ceil(boundary);
            double sd_diff_above = getSD(above + 1) - getSD(above);
            double sd_diff_below = getSD(below) - getSD(below - 1);
            if ((sd_diff_above > 0) ^ (sd_diff_below > 0) &&
                (sd_diff_above >= 0) ^ (sd_diff_below >= 0))
            {
                setMedial(above);
                setMedial(below);
                return true;
            }
        }
        return false;
    }

    void deallocDoubleMatrix(double **matrix, size_t x)
    {
        for (size_t ix = 0; ix < x; ix++)
        {
            free(matrix[ix]);
        }
        free(matrix);
    }

    double **allocDoubleMatrix(size_t x, size_t y)
    {
        double **matrix = (double **)malloc(sizeof(double *) * x);
        for (size_t ix = 0; ix < x; ix++)
        {
            matrix[ix] = (double *)malloc(sizeof(double) * y);
        }
        return matrix;
    }

    int **allocIntMatrix(size_t x, size_t y)
    {
        int **matrix = (int **)malloc(sizeof(int *) * x);
        for (size_t ix = 0; ix < x; ix++)
        {
            matrix[ix] = (int *)malloc(sizeof(int) * y);
        }
        return matrix;
    }

    // TODO: go to commit 79665b21b9e5c6a85a5d to test SDF calculations without object bookkeeping.
    void calculateSignedDistance(double resolution, std::function<bool(double, double, int&)> isValid)
    {
        // Get the size of the grids setup.
        df_resolution_ = resolution;
        // Go a little higher for valid entries on the upper side.
        num_entries_[0] = ceil((maxX_ - minX_) / df_resolution_) + 1;
        num_entries_[1] = ceil((maxY_ - minY_) / df_resolution_) + 1;

        signed_distance_ = allocDoubleMatrix(num_entries_[0], num_entries_[1]);
        double **signed_distance_prev = allocDoubleMatrix(num_entries_[0], num_entries_[1]);
        dist_from_medial_axis_ = allocDoubleMatrix(num_entries_[0], num_entries_[1]);
        double **dist_from_medial_axis_prev = allocDoubleMatrix(num_entries_[0], num_entries_[1]);
        double **inv_df = allocDoubleMatrix(num_entries_[0], num_entries_[1]);
        double **inv_df_prev = allocDoubleMatrix(num_entries_[0], num_entries_[1]);
        double **temp;
        int **temp_int;
        int **objs = allocIntMatrix(num_entries_[0], num_entries_[1]);
        int **objs_prev = allocIntMatrix(num_entries_[0], num_entries_[1]);

        // Will use the null func a lot.
        auto null_func = [](size_t i, size_t j){};

        std::cout << "Allocated memory" << std::endl;

        // Start off by computing a binary image (0 if obstacle, inf if none)
        // and it's reversal (inf if obstacle and 0 if none).
        for (unsigned int ix = 0; ix < num_entries_[0]; ix++)
        {
            for (unsigned int iy = 0; iy < num_entries_[1]; iy++)
            {
                double x = minX_ + ix * df_resolution_;
                double y = minY_ + iy * df_resolution_;
                int obj_ref;
                if (isValid(x, y, obj_ref))
                {
                    signed_distance_[ix][iy] = std::numeric_limits<double>::max();
                    inv_df[ix][iy] = 0;
                    objs[ix][iy] = -1; // -1 means no object yet.
                }
                else
                {
                    signed_distance_[ix][iy] = 0;
                    inv_df[ix][iy] = std::numeric_limits<double>::max();
                    objs[ix][iy] = obj_ref;
                }
                dist_from_medial_axis_[ix][iy] = std::numeric_limits<double>::max();
            }
        }
        std::cout << "Calculated Binary image." << std::endl;

        temp = signed_distance_prev;
        signed_distance_prev = signed_distance_;
        signed_distance_ = temp;

        temp = inv_df_prev;
        inv_df_prev = inv_df;
        inv_df = temp;

        temp_int = objs_prev;
        objs_prev = objs;
        objs = temp_int;

        // Now, compute the 1D transform along each row of the grid.
        for (unsigned int ix = 0; ix < num_entries_[0]; ix++)
        {
            auto sdf_get = [signed_distance_prev, ix](size_t i){ return signed_distance_prev[ix][i]; };
            auto sdf_set = [this, ix](size_t i, double toSet)
            { 
                this->signed_distance_[ix][i] = toSet; 
            };

            auto obj_set_at_from = [objs_prev, objs, ix](size_t i, size_t j)
            {
                objs[ix][i] = objs_prev[ix][j];
            };

            std::vector<double> z =
                    distance_transform_one_dimension(sdf_get, sdf_set, obj_set_at_from, num_entries_[1]);

            //auto medial_get = [this, ix](unsigned int i){ return sqrt(this->signed_distance_[ix][i]); };
            //auto medial_set = [this, ix](unsigned int i){ this->dist_from_medial_axis_[ix][i] = 0.0; };

            //for (auto boundary : z)
            //{
            //    setMedialAxis(boundary, num_entries_[1], medial_get, medial_set);
            //}

            auto inv_get = [inv_df_prev, ix](unsigned int i) { return inv_df_prev[ix][i]; };
            auto inv_set = [inv_df, ix](unsigned int i, double toSet)
            {
                inv_df[ix][i] = toSet;
            };
            distance_transform_one_dimension(inv_get, inv_set, null_func, num_entries_[1]);
        }
        std::cout << "1D x axis done." << std::endl;

        temp = signed_distance_prev;
        signed_distance_prev = signed_distance_;
        signed_distance_ = temp;

        temp = inv_df_prev;
        inv_df_prev = inv_df;
        inv_df = temp;

        temp_int = objs_prev;
        objs_prev = objs;
        objs = temp_int;

        // Now, compute the 1D transform along each column of the grid.
        for (unsigned iy = 0; iy < num_entries_[1]; iy++)
        {
            auto sdf_get = [signed_distance_prev, iy](unsigned int i)
            {
                return signed_distance_prev[i][iy];
            };
            auto sdf_set = [this, iy](unsigned int i, double toSet) 
            { 
                this->signed_distance_[i][iy] = toSet; 
            };
            auto obj_set_at_from = [objs_prev, objs, iy](size_t i, size_t j)
            {
                objs[i][iy] = objs_prev[j][iy];
            };
            std::vector<double> z =
                    distance_transform_one_dimension(sdf_get, sdf_set, obj_set_at_from, num_entries_[0]);

            auto medial_get = [objs, iy](unsigned int i){ return objs[i][iy]; };
            auto medial_set = [this, iy](unsigned int i){ this->dist_from_medial_axis_[i][iy] = 0.0; };

            for (auto boundary : z)
            {
                setMedialAxisObj(boundary, num_entries_[0], medial_get, medial_set);
            }
            auto inv_get = [inv_df_prev, iy](unsigned int i) { return inv_df_prev[i][iy]; };
            auto inv_set = [inv_df, iy](unsigned int i, double toSet)
            {
                inv_df[i][iy] = toSet;
            };
            distance_transform_one_dimension(inv_get, inv_set, null_func, num_entries_[0]);
        }
        std::cout << "1D y axis done." << std::endl;

        for (unsigned ix = 0; ix < num_entries_[0]; ix++)
        {
            auto sdf_get = [signed_distance_prev, ix](unsigned int i)
            {
                return signed_distance_prev[ix][i];
            };
            auto sdf_set = [this, ix](unsigned int i, double toSet) 
            { 
                //this->signed_distance_[ix][i] = toSet; 
            };
            auto obj_set_at_from = [objs_prev, objs](size_t i, size_t j)
            {
                //objs[ix][i] = objs_prev[ix][j];
            };
            std::vector<double> z =
                    distance_transform_one_dimension(sdf_get, sdf_set, obj_set_at_from, num_entries_[1]);
            auto medial_get = [objs, ix](unsigned int i){ return objs[ix][i]; };
            auto medial_set = [this, ix](unsigned int i){ this->dist_from_medial_axis_[ix][i] = 0.0; };

            for (auto boundary : z)
            {
                setMedialAxisObj(boundary, num_entries_[1], medial_get, medial_set);
            }
        }

        // Done with the standard SDF! Finish the last computations.
        for (unsigned int ix = 0; ix < num_entries_[0]; ix++)
        {
            for (unsigned int iy = 0; iy < num_entries_[1]; iy++)
            {
                signed_distance_[ix][iy] = sqrt(signed_distance_[ix][iy]) - sqrt(inv_df[ix][iy]);
            }
        }
        std::cout << "Sqrts done." << std::endl;

        temp = dist_from_medial_axis_prev;
        dist_from_medial_axis_prev = dist_from_medial_axis_;
        dist_from_medial_axis_ = temp;

        // Now, do the same for the medial axis!
        for (unsigned int ix = 0; ix < num_entries_[0]; ix++)
        {
            auto ma_get = [dist_from_medial_axis_prev, ix](unsigned int i) 
            {
                return dist_from_medial_axis_prev[ix][i]; 
            };
            auto ma_set = [this, ix](unsigned int i, double toSet)
            {
                this->dist_from_medial_axis_[ix][i] = toSet;
            };
            distance_transform_one_dimension(ma_get, ma_set, null_func, num_entries_[1]);
        }
        std::cout << "Medial 1D x axis done." << std::endl;

        temp = dist_from_medial_axis_prev;
        dist_from_medial_axis_prev = dist_from_medial_axis_;
        dist_from_medial_axis_ = temp;

        for (unsigned int iy = 0; iy < num_entries_[1]; iy++)
        {
            auto ma_get = [dist_from_medial_axis_prev, iy](unsigned int i) 
            {
                return dist_from_medial_axis_prev[i][iy]; 
            };
            auto ma_set = [this, iy](unsigned int i, double toSet)
            {
                this->dist_from_medial_axis_[i][iy] = toSet;
            };
            distance_transform_one_dimension(ma_get, ma_set, null_func, num_entries_[0]);
        }
        std::cout << "Medial 1D x axis done." << std::endl;

        for (unsigned int ix = 0; ix < num_entries_[0]; ix++)
        {
            for (unsigned int iy = 0; iy < num_entries_[1]; iy++)
            {
                dist_from_medial_axis_[ix][iy] = sqrt(dist_from_medial_axis_[ix][iy]);
            }
        }
        std::cout << "Medial sqrts done." << std::endl;
    }

    // TODO: we can write out the streams, but can we read them back in?
    bool writeDfToJsonFile(double **data, std::string filename) const
    {
        std::ofstream outfile;
        outfile.open(filename);
        bool toReturn = writeDfToJsonToStream(data, outfile);
        outfile.close();
        return toReturn;
    }

    bool writeIntsToJsonFile(int **data, std::string filename) const
    {
        std::ofstream outfile;
        outfile.open(filename);
        bool toReturn = writeDfToJsonToStream(data, outfile);
        outfile.close();
        return toReturn;
    }

    template <class T>
    bool writeDfToJsonToStream(T **data, std::ostream& os) const
    {
        os << "{";
        os << "\"resolution\": " << df_resolution_ << "," << std::endl;
        os << "\"field\": [" << std::endl; 
        for (unsigned int ix = 0; ix < num_entries_[0]; ix++)
        {
            os << "[";
            for (unsigned int iy = 0; iy < num_entries_[1]; iy++)
            {
                os << data[ix][iy];
                if (iy < num_entries_[1] - 1) {
                    os << ",";
                }
            }
            os << "]";
            if (ix < num_entries_[0] - 1)
            {
                os << ",";
            }
            os << std::endl;
        }
        os << "]}";
        os.flush();
        return true;
    }

    double minX_;
    double maxX_;
    double minY_;
    double maxY_;

    double df_resolution_; // Distance field resolution.
    double **signed_distance_; // The signed distance of each position.
    double **dist_from_medial_axis_; // The distance from the medial axis.
    size_t num_entries_[2]; // The number of enteries in each dimension, or ceil((max__ - min__) / df_resolution).
};

#endif