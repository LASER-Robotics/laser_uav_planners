/**
 * @file common.hpp
 * @author Krystof Teissing (teisskry@gmail.com)
 * @version 0.1
 * @date 2023-05-25
 * 
 */

#pragma once

#include <Eigen/Dense>

namespace pmm {

// TYPES -----------------------------------------------------------------/
// Define the scalar type used.
using Scalar = double;
static constexpr Scalar INF = std::numeric_limits<Scalar>::infinity();

// Define `Dynamic` matrix size.
static constexpr int Dynamic = Eigen::Dynamic;

// Using shorthand for `Matrix<rows, cols>` with scalar type.
template<int rows = Dynamic, int cols = rows>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

// Using shorthand for `Vector<rows>` with scalar type.
template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;

// Using shorthand for `Array<rows, cols>` with scalar type.
template<int rows = Dynamic, int cols = rows>
using Array = Eigen::Array<Scalar, rows, cols>;

template<int rows = Dynamic>
using ArrayVector = Array<rows, 1>;

// STRUCTS ----------------------------------------------------------------/
struct QuadState {
  Vector<3> p;
  Vector<3> v;
  Vector<3> a;
  void setZero(){
    p.setZero();
    v.setZero();
  }
};

// GRAVITY ----------------------------------------------------------------/
// Gravity Value [m/s^2]
extern Scalar G;

// Gravity Vector [m/s^2]
extern Vector<3> GVEC;

// DRAG -------------------------------------------------------------------/
extern Eigen::Matrix3d D_coeffs;

void redistribute(Eigen::Vector3d& vec);
} // namespace pmm
