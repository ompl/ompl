/***************************************************************************
 *   Copyright (C) 2016 by Саша Миленковић                                 *
 *   sasa.milenkovic.xyz@gmail.com                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *   ( http://www.gnu.org/licenses/gpl-3.0.en.html )                       *
 *                     *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*
 * Modified by Gaurav Jalan @HiPeRLab Berkeley
 * Changed function arguments
 * Added new Quartic namespace
 */

#ifndef QUARTIC_H_INCLUDED
#define QUARTIC_H_INCLUDED

#include <complex>

namespace Quartic {

const double PI = 3.141592653589793238463L;
const double M_2PI = 2*PI;
const double eps=1e-12;

typedef std::complex<double> DComplex;

//---------------------------------------------------------------------------
// useful for testing
 inline DComplex polinom_2(DComplex x, double a, double b)
 {
   //Horner's scheme for x*x + a*x + b
   return x * (x + a) + b;
 }

//---------------------------------------------------------------------------
// useful for testing
 inline DComplex polinom_3(DComplex x, double a, double b, double c)
 {
   //Horner's scheme for x*x*x + a*x*x + b*x + c;
   return x * (x * (x + a) + b) + c;
 }

//---------------------------------------------------------------------------
// useful for testing
 inline DComplex polinom_4(DComplex x, double a, double b, double c, double d)
 {
   //Horner's scheme for x*x*x*x + a*x*x*x + b*x*x + c*x + d;
   return x * (x * (x * (x + a) + b) + c) + d;
 }

//---------------------------------------------------------------------------
// x - array of size 3
// In case 3 real roots: => x[0], x[1], x[2], return 3
//         2 real roots: x[0], x[1],          return 2
//         1 real root : x[0], x[1] ± i*x[2], return 1
unsigned int solveP3(double a, double b, double c, double* x);

//---------------------------------------------------------------------------
// solve quartic equation x^4 + a*x^3 + b*x^2 + c*x + d
// Attention - this function returns dynamically allocated array. It has to be released afterwards.
size_t solve_quartic(const double& a, const double& b,
                        const double& c, const double& d,
                        double root[]);


#endif // QUARTIC_H_INCLUDED
}