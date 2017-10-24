//
// Created by brainslush on 23/10/17.
//

#ifndef CELLFORMATION_EIGEN_H
#define CELLFORMATION_EIGEN_H

using namespace Eigen;

template<typename T>
static inline Quaternion<T> operator+(const Quaternion<T> &lhs, const Quaternion<T> &rhs) {
    return Quaternion<T>(
            lhs.w() + rhs.w(),
            lhs.x() + rhs.x(),
            lhs.y() + rhs.y(),
            lhs.z() + rhs.z());
}

template<typename T>
static inline Quaternion<T> operator*(const Quaternion<T> &lhs, T rhs) {
    return Eigen::Quaternion<T>(
            lhs.w() * rhs,
            lhs.x() * rhs,
            lhs.y() * rhs,
            lhs.z() * rhs);
}

#endif //CELLFORMATION_EIGEN_H
