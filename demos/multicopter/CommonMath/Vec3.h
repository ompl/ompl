/*!
 * Rapid Collision Detection for Multicopter Trajectories
 *
 * Copyright 2019 by High Performance Robotics Lab, UC Berkeley
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <assert.h>
#include <math.h>
#include <limits>

namespace CommonMath {

class Vec3;
inline Vec3 operator*(const double lhs, const Vec3 rhs);
inline Vec3 operator*(const Vec3 lhs, const double rhs);
inline Vec3 operator*(const Vec3 lhs, const int rhs);
inline Vec3 operator*(const int lhs, const Vec3 rhs);

//! 3D vector class with common vector operations.
class Vec3 {
 public:
  double x, y, z;  //!< the three components of the vector

  Vec3(void)
      : x(std::numeric_limits<double>::quiet_NaN()),
        y(std::numeric_limits<double>::quiet_NaN()),
        z(std::numeric_limits<double>::quiet_NaN()) {
  }  //!<Initialises all members to NaN
  Vec3(double xin, double yin, double zin)
      : x(xin),
        y(yin),
        z(zin) {
  }  //!< Initialise vector

  Vec3(const double in[3])
      : x(in[0]),
        y(in[1]),
        z(in[2]) {
  }  //!< Initialise vector

  Vec3(Vec3 const &in)
      : x(in.x),
        y(in.y),
        z(in.z) {
  }  //!< Initialise from Vec3

  //!Getter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline double operator[](int i) const {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    //we're doing something wrong if we get here
    assert(0);
    return std::numeric_limits<double>::quiet_NaN();
  }

  //!Setter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline double & operator[](int i) {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    //we're doing something wrong if we get here
    assert(0);
    //fail loudly:
    x = y = z = std::numeric_limits<double>::quiet_NaN();
    return x;
  }

  //!Calculate the dot product of two vectors
  inline double Dot(const Vec3 rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  }

  //!Calculate the cross product of two vectors
  inline Vec3 Cross(const Vec3 rhs) const {
    return Vec3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z,
                x * rhs.y - y * rhs.x);
  }

  //!Calculate the Euclidean norm of the vector, squared (= sum of squared elements).
  inline double GetNorm2Squared(void) const {
    return this->Dot(*this);
  }

  //!Calculate the Euclidean norm of the vector.
  inline double GetNorm2(void) const {
    return sqrt(GetNorm2Squared());
  }

  //!Get the unit vector pointing along the same direction as this vector. Will fail for zero vectors.
  inline Vec3 GetUnitVector(void) const {
    float const n = this->GetNorm2();
    return (*this) / n;
  }

  inline Vec3& operator=(const Vec3& other) {
    if (this != &other) {
      x = other.x;
      y = other.y;
      z = other.z;
    }
    return *this;
  }

  inline Vec3 operator+(const Vec3 rhs) const {
    return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
  }
  inline Vec3 operator-(const Vec3 rhs) const {
    return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
  }
  inline Vec3 operator/(const double rhs) const {
    return Vec3(x / rhs, y / rhs, z / rhs);
  }
  inline Vec3 operator+() const {
    return (*this);
  }  //mostly used to write things prettily, contrasting to the below.
  inline Vec3 operator-() const {
    return (*this) * double(-1);
  }

  inline Vec3 operator+=(const Vec3 rhs) {
    (*this) = (*this) + rhs;
    return *this;
  }
  inline Vec3 operator-=(const Vec3 rhs) {
    (*this) = (*this) - rhs;
    return *this;
  }
  inline Vec3 operator*=(const double &rhs) {
    *this = (*this) * rhs;
    return *this;
  }
  inline Vec3 operator/=(const double &rhs) {
    *this = (*this) / rhs;
    return *this;
  }
};

//Multiply with scalar of double type:
inline Vec3 operator*(const double lhs, const Vec3 rhs) {
  return Vec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}
inline Vec3 operator*(const Vec3 lhs, const double rhs) {
  return Vec3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

//Multiply with scalar of integer type:
inline Vec3 operator*(const Vec3 lhs, const int rhs) {
  return Vec3(rhs * lhs.x, rhs * lhs.y, rhs * lhs.z);
}

inline Vec3 operator*(const int lhs, const Vec3 rhs) {
  return Vec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

}