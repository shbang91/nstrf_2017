#ifndef MYDRAKE_ROTMATQUAT_H
#define MYDRAKE_ROTMATQUAT_H
#include <cmath>
#include <Eigen/Dense>

#include "my_drake/eigen_types.h"

namespace my_drake {
namespace math {
/**
 * Computes one of the quaternion from a rotation matrix.
 * This implementation is adapted from simbody
 * https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
 * Notice that there are two quaternions corresponding to the same rotation,
 * namely `q` and `-q` represent the same rotation.
 * @param M A 3 x 3 rotation matrix.
 * @return a 4 x 1 unit length vector, the quaternion corresponding to the
 * rotation matrix.
 */
template <typename Derived>
Vector4<typename Derived::Scalar> rotmat2quat(
    const Eigen::MatrixBase<Derived>& M) {

  typedef typename Derived::Scalar Scalar;

  Vector4<Scalar> q;

  // Check if the trace is larger than any diagonal
  Scalar tr = M.trace();
  if (tr >= M(0, 0) && tr >= M(1, 1) && tr >= M(2, 2)) {
    q(0) = 1 + tr;
    q(1) = M(2, 1) - M(1, 2);
    q(2) = M(0, 2) - M(2, 0);
    q(3) = M(1, 0) - M(0, 1);
  } else if (M(0, 0) >= M(1, 1) && M(0, 0) >= M(2, 2)) {
    q(0) = M(2, 1) - M(1, 2);
    q(1) = Scalar(1) - (tr - 2 * M(0, 0));
    q(2) = M(0, 1) + M(1, 0);
    q(3) = M(0, 2) + M(2, 0);
  } else if (M(1, 1) >= M(2, 2)) {
    q(0) = M(0, 2) - M(2, 0);
    q(1) = M(0, 1) + M(1, 0);
    q(2) = Scalar(1) - (tr - 2 * M(1, 1));
    q(3) = M(1, 2) + M(2, 1);
  } else {
    q(0) = M(1, 0) - M(0, 1);
    q(1) = M(0, 2) + M(2, 0);
    q(2) = M(1, 2) + M(2, 1);
    q(3) = 1 - (tr - 2 * M(2, 2));
  }
  Scalar scale = q.norm();
  q /= scale;
  return q;
}

}  // namespace math
}  // namespace my_drake


#endif