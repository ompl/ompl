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

/* Author: Mark Moll */

#include "ompl/base/spaces/TrochoidStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;

namespace
{
    constexpr double twopi = 2. * boost::math::constants::pi<double>();
    constexpr double onepi = boost::math::constants::pi<double>();
    const double TROCHOID_EPS = 1e-6;
    const double TROCHOID_ZERO = -1e-7;

    inline double mod2pi(double x)
    {
        if (x < 0 && x > TROCHOID_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * TROCHOID_EPS)
            xm = 0.;
        return xm;
    }

    bool checkConditionsBSB(double delta_1, double delta_2, double tA, double tB, double xt10, double yt10, double phit1, double xt20, double yt20, double phit2, double radius, double wind_ratio, TrochoidStateSpace::TrochoidPath &trochoid_path) {
        // check that tA, tB, and T are valid numbers 
        if ( std::isnan(tA) || std::isnan(tB) ){
            return false;
        }

        double xtAdot = cos(delta_1*(1./radius)*tA + phit1) + wind_ratio;
        double ytAdot = sin(delta_1*(1./radius)*tA + phit1);
        double xtBdot = cos(delta_2*(1./radius)*tB + phit2) + wind_ratio;
        double ytBdot = sin(delta_2*(1./radius)*tB + phit2);

        double xtA = (radius/delta_1)*sin(delta_1*(1./radius)*tA + phit1) + wind_ratio*tA + xt10;
        double ytA = (-radius/delta_1)*cos(delta_1*(1./radius)*tA + phit1) + yt10;
        double xtB = (radius/delta_2) *sin(delta_2*(1./radius)*tB + phit2) + wind_ratio*tB + xt20;
        double ytB = (-radius/delta_2)*cos(delta_2*(1./radius)*tB + phit2) + yt20;

        double eps = 0.001;
        double t2pi = 2. * onepi * radius;

        if ( tB < -t2pi || tB > t2pi ){
            return false;
        } 
        if ( tA < 0 || tA > 2*t2pi ){
            return false;
        }
        if ( (xtB-xtA)*xtAdot < -eps ){
            return false;
        }  
        if ( (ytB-ytA)*ytAdot < -eps ){
            return false;
        } 
        if ( (xtB-xtA)*xtBdot < -eps ){
            return false;
        }  
        if ( (ytB-ytA)*ytBdot < -eps ){
            return false;
        } 
        // check that equal 
        if ( fabs( (xtB - xtA)/(ytB-ytA) - xtAdot/ytAdot ) >= eps ){
            return false;
        }  
        if ( fabs( (xtB - xtA)/(ytB-ytA) - xtBdot/ytBdot ) >= eps ){
            return false;
        }  
        // check endpoint
        double p = sqrt( (xtB-xtA)*(xtB-xtA) + (ytB-ytA)*(ytB-ytA) )/sqrt(xtBdot*xtBdot + ytBdot*ytBdot);
        double T = tA + p + (t2pi - tB);  

        if (T < trochoid_path.length()) {//Update path if it is shorter
            trochoid_path.length_[0] = tA;
            trochoid_path.length_[1] = p;
            trochoid_path.length_[2] = t2pi - tB;
        }

        return true;
    }

    double fixedPointBSB( double p0, double delta_1, double delta_2, double k, double xt10, double yt10, double phit1, double xt20, double yt20, double phit2, double radius, double wind_ratio )
    {
        double tol = 0.0001;
        int N = 50;
        std::vector<double> pvec(N);
        pvec[0] = p0;
        int i = 1;
        double omega = 1./radius;
        while (i <= N){
            double p = pvec[i-1];
            double E = (delta_1-delta_2) / ( delta_2*delta_1*omega ) - (yt20-yt10)/wind_ratio;
            double F = (xt20-xt10)/wind_ratio + (delta_1/delta_2-1)*p + (phit1-phit2 + 2.0*k*onepi) 
                                                                /(delta_2*omega);
            double G = (yt20 - yt10) + 1.0/wind_ratio*(delta_2-delta_1)/(delta_1*delta_2*omega);
            double f = E*cos(delta_1*omega*p + phit1) + F*sin(delta_1*omega*p + phit1) - G;
            double fBar = - E*sin(delta_1*omega*p + phit1)*delta_1*omega 
                        + F*cos(delta_1*omega*p + phit1)*delta_1*omega
                        + sin(delta_1*omega*p + phit1)*(delta_1/delta_2-1);
            pvec[i] = p - f/fBar;
            if ( fabs( pvec[i] - pvec[i-1]) < tol ){
            return pvec[i];
            }
            i++;
        }
        return std::numeric_limits<double>::quiet_NaN();
    }



    void trochoidBSB(double x0, double y0, double phi0, double xf, double yf, double phif, double delta_1, double delta_2, double radius, double wind_ratio, TrochoidStateSpace::TrochoidPath &path)
    {

        double V_w = wind_ratio;
        double omega = 1.0/radius;
        double t2pi = 2 * onepi * radius;

        double phit1 = fmod(phi0, 2. * onepi);
        double xt10 = x0 - delta_1 * radius * std::sin(phit1);
        double yt10 = y0 + delta_1 * radius * std::cos(phit1);

        double phit2 = fmod(phif - delta_2 * omega * t2pi, 2. * onepi);
        double xt20 = xf - delta_2 * radius * std::sin(delta_2 * omega * t2pi + phit2) - V_w * t2pi;
        double yt20 = yf + delta_2 * radius * cos(delta_2 * omega * t2pi + phit2);

        bool solnFound = false;

        if (delta_1 == delta_2) { // RSR/LSL: analytic solution
            for (int k_int = -2; k_int < 2; k_int++){
                double k = (double)k_int;
                double alpha = std::atan2(yt20 - yt10, xt20 - xt10 + V_w*( fmod(phit1-phit2, 2.0*onepi) - 2.0*k*onepi )/( delta_1*omega ));
                double tA = t2pi/(delta_1*2.0*onepi)*( std::asin(V_w*std::sin(alpha)) + alpha - phit1 );
                if ( tA > t2pi || tA < 0){
                    tA = tA - t2pi*floor(tA/t2pi);
                }
                double tB = tA + ( fmod(phit1 - phit2, 2.0*onepi) - 2.0*k*onepi ) / (delta_1*2.0*onepi) * t2pi;
                if ( checkConditionsBSB(delta_1, delta_2, tA, tB, xt10, yt10, phit1, xt20, yt20, phit2, radius, wind_ratio, path) ){
                    solnFound = true;
                }
            }
        } else {  // RSL/LSR: numerical solution
            for (int k_int = -2; k_int < 2; k_int++){
                double k = (double)k_int;
                int numTestPts = 10;
                std::vector<double> rootsComputed(numTestPts);
                double sameRootEpsilon = 0.001;
                for ( int l = 0; l < numTestPts; l++){
                    bool rootAlreadyfound = false;
                    // initial guess
                    double p0 = (double)(l+1) * t2pi / (double)numTestPts;
                    // root solving 
                    double tA = fixedPointBSB( p0, delta_1, delta_2, k, xt10, yt10, phit1, xt20, yt20, phit2, radius, wind_ratio);
                    // check bounds
                    if ( tA > t2pi || tA < 0){
                        tA = tA - t2pi*floor(tA/t2pi);
                    }
                    // check if the root has already been computed 
                    if ( l > 0 ){
                    for (int i = 0; i < l; i++){
                        if ( fabs( rootsComputed[i] - tA ) <= sameRootEpsilon ){
                        rootAlreadyfound = true;
                        } 
                    }
                    }
                    // store the computed root
                    rootsComputed[l] = tA;
                    // if the root is unique
                    if ( !rootAlreadyfound ){
                        double tB = delta_1/delta_2*tA + (phit1 - phit2 + 2.0*k*onepi)/(delta_2*omega);
                        if ( checkConditionsBSB(delta_1, delta_2, tA, tB, xt10, yt10, phit1, xt20, yt20, phit2, radius, wind_ratio, path) ){
                            solnFound = true;
                        }
                    }
                }
            }
        }

        return;
    }

    bool checkConditionsBBB( double delta_1, double tA, double tB, double T, double xt10, double yt10, double phit1, double xf, double yf, double phif, double radius, double wind_ratio, TrochoidStateSpace::TrochoidPath &trochoid_path){

        if ( std::isnan(tA) || std::isnan(tB) || std::isnan(T) ){
            return false;
        }
        double omega = 1./radius;
        double t2pi = 2 * onepi * radius;
        double d2 = -delta_1;
        double d3 = delta_1;

        double phit3 = fmod ( phif - d3*omega*T , 2.0*onepi);
        double xt30 = xf - 1.0/(d3*omega)*sin( phif ) - wind_ratio*T;
        double yt30 = yf + 1.0/(d3*omega)*cos( phif );

        double phit2 = 2*delta_1*omega*tA + phit1;
        double xt20 = xt30 - 2*1.0/(d2*omega)*sin(d2*omega*tB + phit2);
        double yt20 = yt30 + 2*1.0/(d2*omega)*cos(d2*omega*tB + phit2);


        double xt1tA = 1.0/(delta_1*omega)*sin(delta_1*omega*tA + phit1)
                                    + wind_ratio*tA + xt10;
        double yt1tA = -1.0/(delta_1*omega)*cos(delta_1*omega*tA + phit1) + yt10;

        double xt2tA = 1.0/(d2*omega)*sin(d2*omega*tA + phit2)
                                    + wind_ratio*tA + xt20;
        double yt2tA = -1.0/(d2*omega)*cos(d2*omega*tA + phit2) + yt20;

        double xt2tB = 1.0/(d2*omega)*sin(d2*omega*tB + phit2)
                                    + wind_ratio*tB + xt20;
        double yt2tB = -1.0/(d2*omega)*cos(d2*omega*tB + phit2) + yt20;
        double xt3tB = 1.0/(d3*omega)*sin(d3*omega*tB + phit3)
                                    + wind_ratio*tB + xt30;
        double yt3tB = -1.0/(d3*omega)*cos(d3*omega*tB + phit3) + yt30;  

        double xt1dottA = 1.0*cos(delta_1*omega*tA + phit1) + wind_ratio;
        double yt1dottA = 1.0*sin(delta_1*omega*tA + phit1);
        double xt2dottA = 1.0*cos(d2*omega*tA + phit2) + wind_ratio;
        double yt2dottA = 1.0*sin(d2*omega*tA + phit2);

        double xt2dottB= 1.0*cos(d2*omega*tB + phit2) + wind_ratio;
        double yt2dottB = 1.0*sin(d2*omega*tB + phit2);
        double xt3dottB = 1.0*cos(d3*omega*tB + phit3) + wind_ratio;
        double yt3dottB = 1.0*sin(d3*omega*tB + phit3);

        double eps = 0.001;


        // check that: 0 <= tA <= tB <= T <= 4.0*t2pi   
        if ( tA < 0 || tB < tA || T < tB || T > 4.0*t2pi ){
            return false;
        }
        if ( fabs(xt1tA - xt2tA) >= eps ){
            return false; 
        }
        if ( fabs(yt1tA - yt2tA) >= eps ){
            return false; 
        }
        if ( fabs(xt2tB - xt3tB) >= eps ){
            return false; 
        }
        if ( fabs(yt2tB - yt3tB) >= eps ){
            return false; 
        }
        if ( fabs(xt1dottA - xt2dottA) >= eps ){
            return false; 
        }
        if ( fabs(yt1dottA - yt2dottA) >= eps ){
            return false; 
        }
        if ( fabs(xt2dottB - xt3dottB) >= eps ){
            return false; 
        }
        if ( fabs(yt2dottB - yt3dottB) >= eps ){
            return false; 
        }

        if (T < trochoid_path.length()) {//Update path if it is shorter
            trochoid_path.length_[0] = tA;
            trochoid_path.length_[1] = tB - tA;
            trochoid_path.length_[2] = T - tB;
        }

        return true;
    }

    void fixedpointBBB(double p0, double p1, double delta_1, double k, double xt10, double yt10, double phit1, double xf, double yf, double phif, double radius, double wind_ratio, std::vector<double> &pvecOut) {
        ///TODO: Implement TrochoidBBB
        pvecOut.resize(2);
        double tol = 0.0001;
        int N = 50;
        double d2 = -delta_1;
        double d3 = delta_1;
        double omega = 1.0/radius;
        std::vector<double> pvec1(N);
        std::vector<double> pvec2(N);
        pvec1[0] = p0;
        pvec2[0] = p1;
        int i = 1;
        while (i <= N){
            double tA = pvec1[i-1];
            double T = pvec2[i-1];
            // vector F = [f1; f2];
            double f1 = 2./(delta_1*omega)*sin(delta_1*omega*tA+phit1) + wind_ratio*T +xt10 - xf + 1.0/(d3*omega)*sin(phif) + 2/(d2*omega)*sin( d2*omega*T/2 + (phif+phit1 + k*2*onepi)/2 
                            + delta_1*omega*tA );
            double f2 = -2*1.0/(delta_1*omega)*cos(delta_1*omega*tA+phit1) 
                        + yt10 - yf - 1.0/(d3*omega)*cos(phif) 
                        - 2*1.0/(d2*omega)
                        *cos( d2*omega*T/2 + (phif+phit1 + k*2*onepi)/2 
                                + delta_1*omega*tA );
            std::vector<double> F(2);
            // matrix FBar = [ a b ; c d ];
            double a = 2*1.0*( cos(delta_1*omega*tA + phit1) 
                                    - cos( d2*omega*T/2 + (phif+phit1 + k*2*onepi)/2 
                                            + delta_1*omega*tA ) );
            double b = wind_ratio+1.0*cos( d2*omega*T/2 
                                                + (phif+phit1 + k*2*onepi)/2 
                                                + delta_1*omega*tA );
            double c = 2*1.0*( sin(delta_1*omega*tA + phit1) 
                                    - sin( d2*omega*T/2 + (phif+phit1 + k*2*onepi)/2 
                                            + delta_1*omega*tA ) );
            double d = 1.0*sin( d2*omega*T/2 + (phif+phit1 + k*2*onepi)/2 
                                    + delta_1*omega*tA );
            double det = a*d - b*c;
            // pvec = x - FBar*FBarInv;
            pvec1[i] = pvec1[i-1] - (d/det*f1 - b/det*f2);
            pvec2[i] = pvec2[i-1] - (-c/det*f1 + a/det*f2);
            // convergence criteria
            if ( (pvec1[i]-pvec1[i-1])*(pvec1[i]-pvec1[i-1]) 
                + (pvec2[i]-pvec2[i-1])*(pvec2[i]-pvec2[i-1]) < tol ){
                pvecOut[0] = pvec1[i];
                pvecOut[1] = pvec2[i];
                return;
            }
            i++;
        }
        pvecOut[0] = std::numeric_limits<double>::quiet_NaN();
        pvecOut[1] = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    void trochoidBBB(double x0, double y0, double phit1, double xf, double yf, double phif, double delta_1, double radius, double wind_ratio, TrochoidStateSpace::TrochoidPath &path) {
        double omega = (1/radius);
        double t2pi = 2.0 * onepi * radius;

        double xt10 = x0 - 1.0/(delta_1*omega)*sin(phit1);
        double yt10 = y0 + 1.0/(delta_1*omega)*cos(phit1);

        double delta_2 = - delta_1;

        // test a grid of initial conditions, grid resolution
        int numTestPts = 10;
        std::vector< std::vector<double> > rootsComputed(numTestPts*numTestPts);
        for (int i = 0; i < rootsComputed.size(); i++){
            rootsComputed[i].resize(3);
        }
        double sameRootEpsilon = 0.001;
        bool solnFound{false};
        for (int kint = -2; kint < 3; kint++){
            double k = (double)kint;
            for ( int l = 0; l < numTestPts; l++){
            for ( int n = 0; n < numTestPts; n++){
                bool rootAlreadyfound = false;
                // initial guess
                std::vector<double> pvecInit(2);
                std::vector<double> pvecOut(2);
                double tA_test = (double)(l+1)*t2pi/numTestPts;
                double T_test = (double)(n+1)*3*t2pi/numTestPts;
                fixedpointBBB(tA_test, T_test, delta_1, k, xt10, yt10, phit1, xf, yf, phif, radius, wind_ratio, pvecOut);
                double tA = pvecOut[0];
                double T = pvecOut[1];
                double tB = tA + T/2 + (phif - phit1 + 2.0*k*onepi)
                                    /( 2*delta_2*omega );

                int curRow = (l)*numTestPts+(n);
                // check if the root has already been computed 
                if ( l > 0 || n > 0 ){
                    for (int i = 0; i < curRow; i++){
                        if (    fabs( rootsComputed[i][0] - tA ) <= sameRootEpsilon 
                            && fabs( rootsComputed[i][1] - tB ) <= sameRootEpsilon 
                            && fabs( rootsComputed[i][2] - T ) <= sameRootEpsilon ){
                        rootAlreadyfound = true;
                        } 
                    }
                }
                // store the computed rootz
                rootsComputed[curRow][0] = tA;
                rootsComputed[curRow][1] = tB;
                rootsComputed[curRow][2] = T;

                // if the root is unique
                if ( !rootAlreadyfound ){
                    if ( checkConditionsBBB(delta_1, tA, tB, T, xt10, yt10, phit1, xf, yf, phif, radius, wind_ratio, path)){
                        solnFound = true;
                    }
                }
            }
            }
        }
        
        // if ( !solnFound ){
        //     switch ( pt ){
        //     case pathType::LRL: LRL_.set_pathStatusInfeasible(); break;
        //     case pathType::RLR: RLR_.set_pathStatusInfeasible(); break;
        //     }
        // }
  
    }


    TrochoidStateSpace::TrochoidPath trochoidRSR(double x0, double y0, double phi0, double xf, double yf, double phif, double radius, double wind_ratio)
    {
        TrochoidStateSpace::TrochoidPath path(TrochoidStateSpace::dubinsPathType[1]);
        trochoidBSB(x0, y0, phi0, xf, yf, phif, -1, -1, radius, wind_ratio, path);
        return path;
    }

    TrochoidStateSpace::TrochoidPath trochoidLSL(double x0, double y0, double phi0, double xf, double yf, double phif, double radius, double wind_ratio)
    {
        TrochoidStateSpace::TrochoidPath path(TrochoidStateSpace::dubinsPathType[0]);
        trochoidBSB(x0, y0, phi0, xf, yf, phif, 1, 1, radius, wind_ratio, path);
        return path; 
    }

    TrochoidStateSpace::TrochoidPath trochoidRSL(double x0, double y0, double phi0, double xf, double yf, double phif, double radius, double wind_ratio)
    {
        TrochoidStateSpace::TrochoidPath path(TrochoidStateSpace::dubinsPathType[2]);
        trochoidBSB(x0, y0, phi0, xf, yf, phif, -1, 1, radius, wind_ratio, path);
        return path;
    }

    TrochoidStateSpace::TrochoidPath trochoidLSR(double x0, double y0, double phi0, double xf, double yf, double phif, double radius, double wind_ratio)
    {
        TrochoidStateSpace::TrochoidPath path(TrochoidStateSpace::dubinsPathType[3]);
        trochoidBSB(x0, y0, phi0, xf, yf, phif, 1, -1, radius, wind_ratio, path);
        return path;
    }

    TrochoidStateSpace::TrochoidPath trochoidLRL(double x0, double y0, double phi0, double xf, double yf, double phif, double radius, double wind_ratio)
    {
        TrochoidStateSpace::TrochoidPath path(TrochoidStateSpace::dubinsPathType[5]);
        trochoidBBB(x0, y0, phi0, xf, yf, phif, 1, radius, wind_ratio, path);
        return path;
    }

    TrochoidStateSpace::TrochoidPath trochoidRLR(double x0, double y0, double phi0, double xf, double yf, double phif, double radius, double wind_ratio)
    {
        TrochoidStateSpace::TrochoidPath path(TrochoidStateSpace::dubinsPathType[4]);
        trochoidBBB(x0, y0, phi0, xf, yf, phif, -1, radius, wind_ratio, path);
        return path;
    }

    TrochoidStateSpace::TrochoidPath trochoid(const double x0, const double y0, const double phi0, const double xf, const double yf, const double phif, double radius, double wind_ratio, double wind_heading)
    {
        // if (d < TROCHOID_EPS && fabs(alpha - beta) < TROCHOID_EPS)
            // return {TrochoidStateSpace::dubinsPathType[0], 0, d, 0};

        TrochoidStateSpace::TrochoidPath path(trochoidLSL(x0, y0, phi0, xf, yf, phif, radius, wind_ratio)), tmp(trochoidRSR(x0, y0, phi0, xf, yf, phif, radius, wind_ratio));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = trochoidRSL(x0, y0, phi0, xf, yf, phif, radius, wind_ratio);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = trochoidLSR(x0, y0, phi0, xf, yf, phif, radius, wind_ratio);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = trochoidRLR(x0, y0, phi0, xf, yf, phif, radius, wind_ratio);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = trochoidLRL(x0, y0, phi0, xf, yf, phif, radius, wind_ratio);
        if ((len = tmp.length()) < minLength)
            path = tmp;
        return path;
    }

}  // namespace

namespace ompl::base
{
    std::ostream &operator<<(std::ostream &os, const TrochoidStateSpace::TrochoidPath &path)
    {
        os << "TrochoidPath[ type=";
        for (unsigned i = 0; i < 3; ++i)
            if (path.type_[i] == TrochoidStateSpace::TROCHOID_LEFT)
                os << "L";
            else if (path.type_[i] == TrochoidStateSpace::TROCHOID_STRAIGHT)
                os << "S";
            else
                os << "R";
        os << ", length=" << path.length_[0] << '+' << path.length_[1] << '+' << path.length_[2] << '=' << path.length()
           << ", reverse=" << path.reverse_ << " ]";
        return os;
    }
}  // namespace ompl::base

const TrochoidStateSpace::TrochoidPathSegmentType TrochoidStateSpace::dubinsPathType[6][3] = {
    {TROCHOID_LEFT, TROCHOID_STRAIGHT, TROCHOID_LEFT},  {TROCHOID_RIGHT, TROCHOID_STRAIGHT, TROCHOID_RIGHT},
    {TROCHOID_RIGHT, TROCHOID_STRAIGHT, TROCHOID_LEFT}, {TROCHOID_LEFT, TROCHOID_STRAIGHT, TROCHOID_RIGHT},
    {TROCHOID_RIGHT, TROCHOID_LEFT, TROCHOID_RIGHT},    {TROCHOID_LEFT, TROCHOID_RIGHT, TROCHOID_LEFT}};

double TrochoidStateSpace::distance(const State *state1, const State *state2) const
{
    return isSymmetric_ ? symmetricDistance(state1, state2, rho_, eta_, psi_w_) : distance(state1, state2, rho_, eta_, psi_w_);
}
double TrochoidStateSpace::distance(const State *state1, const State *state2, double radius, double wind_ratio, double wind_heading)
{
    return trochoid(state1, state2, radius, wind_ratio, wind_heading).length();
}
double TrochoidStateSpace::symmetricDistance(const State *state1, const State *state2, double radius, double wind_ratio, double wind_heading)
{
    return std::min(trochoid(state1, state2, radius, wind_ratio, wind_heading).length(), trochoid(state2, state1, radius, wind_ratio, wind_heading).length());
}

void TrochoidStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    bool firstTime = true;
    TrochoidPath path;
    interpolate(from, to, t, firstTime, path, state);
}

void TrochoidStateSpace::interpolate(const State *from, const State *to, const double t, bool &firstTime,
                                   TrochoidPath &path, State *state) const
{
    if (firstTime)
    {
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = trochoid(from, to);
        if (isSymmetric_)
        {
            TrochoidPath path2(trochoid(to, from));
            if (path2.length() < path.length())
            {
                path2.reverse_ = true;
                path = path2;
            }
        }
        firstTime = false;
    }
    interpolate(from, path, t, state, rho_, eta_, psi_w_);
}

void TrochoidStateSpace::interpolate(const State *from, const TrochoidPath &path, double t, State *state,
                                   double radius, double wind_ratio, double wind_heading) const
{
    auto *s = allocState()->as<StateType>();
    double seg = t * path.length(), phi, v, x_t10, y_t10, delta;
    s->setXY(0., 0.);
    s->setYaw(from->as<StateType>()->getYaw());
    for (unsigned int i = 0; i < 3 && seg > 0; ++i)
    {
        v = std::min(seg, path.length_[i]);
        phi = s->getYaw();
        seg -= v;
        switch (path.type_[i])
        {
            case TROCHOID_LEFT:
                delta = 1;
                x_t10 = s->getX() - (radius* delta) * sin(phi);
                y_t10 = s->getY() + (radius * delta) * cos(phi);
                s->setXY(x_t10 + (radius * delta) * sin(delta *(1.0/radius) * v + phi)  + wind_ratio * v, \
                y_t10 - (radius * delta) * cos(delta *(1.0/radius) * v + phi));
                s->setYaw(phi + delta *(1.0/radius) *v);
                break;
            case TROCHOID_RIGHT:
                delta = -1;
                x_t10 = s->getX() - (radius* delta) * sin(phi);
                y_t10 = s->getY() + (radius * delta) * cos(phi);
                s->setXY(x_t10 + (radius * delta) * sin(delta *(1.0/radius) * v + phi)  + wind_ratio * v, \
                y_t10 - (radius * delta) * cos(delta *(1.0/radius) * v + phi));
                s->setYaw(phi + delta *(1.0/radius) *v);
                break;
            case TROCHOID_STRAIGHT:
                s->setXY(s->getX() + v * cos(phi) + v * wind_ratio, s->getY() + v * sin(phi));
                break;
        }
    }

    ///TODO: Reproject from trochoid space to inertial space
    state->as<StateType>()->setX(s->getX() + from->as<StateType>()->getX());
    state->as<StateType>()->setY(s->getY() + from->as<StateType>()->getX());
    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(s->getYaw());
    freeState(s);
}

unsigned int TrochoidStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

TrochoidStateSpace::TrochoidPath TrochoidStateSpace::trochoid(const State *state1, const State *state2) const
{
    return trochoid(state1, state2, rho_, eta_, psi_w_);
}

TrochoidStateSpace::TrochoidPath TrochoidStateSpace::trochoid(const State *state1, const State *state2, double radius, double wind_ratio, double wind_heading)
{
    const auto *s1 = static_cast<const TrochoidStateSpace::StateType *>(state1);
    const auto *s2 = static_cast<const TrochoidStateSpace::StateType *>(state2);
    double x0 = s1->getX(), y0 = s1->getY(), psi_0 = s1->getYaw();
    double xf = s2->getX(), yf = s2->getY(), psi_f = s2->getYaw();

    double phi0 = psi_0 - wind_heading;
    double phif = psi_f - wind_heading;

    // Transform into trochoid frame
    double xf_trochoid = xf * std::cos(wind_heading) + yf * std::sin(wind_heading);
    double yf_trochoid = -xf * std::sin(wind_heading) + yf * std::cos(wind_heading);

    return ::trochoid(x0, y0, phi0, xf_trochoid, yf_trochoid, phif, radius, wind_ratio, wind_heading);
}

void TrochoidMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<TrochoidStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool TrochoidMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true, firstTime = true;
    TrochoidStateSpace::TrochoidPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool TrochoidMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;
    TrochoidStateSpace::TrochoidPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
