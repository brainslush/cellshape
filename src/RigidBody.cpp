#include "RigidBody.h"

/*
 * RigidBody.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

using namespace physic;

RigidBody3d::RigidBody3d(
        Eigen::Vector3d iX,
        Eigen::Quaterniond iQ, // rotation in lab frame
        Eigen::Matrix3d iI,
        double iM,
        double iEpsilon,
        std::set<functor *> &iFunctors,
) :
        X(iX),
        q(iQ),
        I(iI.inverse().diagonal()),
        M(iM),
        epsilon(iEpsilon),
        functors(iFunctors) {
    v = Eigen::Vector3d(0, 0, 0);
    L = Eigen::Vector3d(0, 0, 0);
    F = Eigen::Vector3d(0, 0, 0);
    T = Eigen::Vector3d(0, 0, 0);
}

RigidBody3d::RigidBody3d(
        Eigen::Vector3d iX,
        Eigen::Vector3d iV,
        Eigen::Quaterniond iQ, // rotation in lab frame
        Eigen::Vector3d iL,
        Eigen::Matrix3d iI,
        double iM,
        double iEpsilon,
        std::set<functor *> &iFunctors
) :
        X(iX),
        v(iV),
        q(iQ),
        L(iL),
        I(iI.inverse().diagonal()),
        M(iM),
        epsilon(iEpsilon),
        functors(iFunctors) {
    F = Eigen::Vector3d(0, 0, 0);
    T = Eigen::Vector3d(0, 0, 0);
}

RigidBody3d::~RigidBody3d() {
}

Eigen::Vector3d &RigidBody3d::get_position() {
    return X;
}

Eigen::Matrix3d RigidBody3d::get_rotationMatrix() {
    return q.toRotationMatrix();
}

Eigen::Quaterniond &RigidBody3d::get_quaternion() {
    return q;
}

Eigen::Vector3d &RigidBody3d::get_velocity() {
    return v;
}

Eigen::Vector3d &RigidBody3d::get_angularMomentum() {
    return L;
}

void RigidBody3d::set_inertia(Eigen::Matrix3d iI) {
    I = iI.inverse().diagonal();
}

void RigidBody3d::set_mass(double &iM) {
    M = iM;
}

void RigidBody3d::add_force(Eigen::Vector3d &iX, Eigen::Vector3d &iF) {
    F += iF;
    T += (iX - X).cross(iF);
}

Eigen::Quaterniond RigidBody3d::qsum(const Eigen::Quaterniond &l, const Eigen::Quaterniond &r) {
    Eigen::Quaterniond c;
    c.coeffs() = l.coeffs() + r.coeffs();
    return c;
}

Eigen::Quaterniond RigidBody3d::qdiff(const Eigen::Quaterniond &l, const Eigen::Quaterniond &r) {
    Eigen::Quaterniond c;
    c.coeffs() = l.coeffs() - r.coeffs();
    return c;
}

Eigen::Quaterniond RigidBody3d::qscale(const double &s, const Eigen::Quaterniond &q) {
    Eigen::Quaterniond c;
    c.coeffs() = s * q.coeffs();
    return c;
}

Eigen::Quaterniond RigidBody3d::vec2quat(const Eigen::Vector3d &v) {
    Eigen::Quaterniond c;
    c.w() = 0;
    c.vec() = v;
    return c;
}