//
//  PolyfitEigen.hpp
//  Polyfit
//
//  Created by Patrick Löber on 23.11.18.
//  Copyright © 2018 Patrick Loeber. All rights reserved.
//
//  Modified by Beverly Xu on 2.21.24 FIXME: Proper MIT Licensing
//
//  Use the Eigen library for fitting: http://eigen.tuxfamily.org
//  See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for different methods


// TODO: Add function to get derivative coefficients

#include "Eigen/Dense"
#include <iostream>
#include <vector>

Eigen::VectorXd polyfit_Eigen(std::vector<double> x, const Eigen::VectorXd& y, int degree) {
    // Construct the Vandermonde matrix using Eigen's Map for vector initialization
    Eigen::MatrixXd A = Eigen::MatrixXd::NullaryExpr(x.size(), degree + 1,
                     [&x, &degree](Eigen::Index i, Eigen::Index j) { return std::pow(x[i], j); });
    // Solve the least-squares problem
    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
}

/**
 * Taking in an input of polyomial coefficients, from leading power to constant (0->n)
 * 
 * Returns a vector of derivative coefficients corresponding to the input antiderivative terms
*/
template<typename T>
std::vector<T> polyDerivatives(std::vector<T>& coefficients){
    std::vector<T> derivativeCoefficients;
    for(size_t degree = 0; degree < coefficients.size(); degree++){
        derivativeCoefficients.push_back((degree) * coefficients.at(degree)); // for ax^(n), derivative coefficient is a*n
    }
    return derivativeCoefficients;
}

/**
 * Taking in an input of polyomial coefficients, from leading power to constant (n->0), and x values that are desired to be evaluated
 * 
 * Returns a vector of y values corresponding to the x values
*/
template<typename T>
std::vector<T> polyVal(std::vector<T>& coefficients, std::vector<T>& xValues){
    std::vector<T> yValues;
    for(T x : xValues){
        T y = (T)(0);
        for(size_t degree = 0; degree < coefficients.size(); degree++){
            y += pow(x, degree) * coefficients.at(degree);    // y = ax^n + bx^(n-1) + .... + c
        }
        yValues.push_back(y);
    }
    return yValues;
}