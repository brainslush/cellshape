#include "RigidBody.h"

/*
 * RigidBody.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

using namespace physic;

RigidBody3d::RigidBody3d() :
        functors(nullptr) {

}

RigidBody3d::RigidBody3d(
        Eigen::Vector3d iX,
        Eigen::Quaterniond iQ, // rotation in lab frame
        Eigen::Matrix3d iI,
        double iM,
        double iEpsilon,
        std::set<functor *> *iFunctors
) :
        X(std::move(iX)),
        q(iQ),
        M(iM),
        epsilon(iEpsilon),
        functors(iFunctors) {
    I = iI.diagonal();
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
        std::set<functor *> *iFunctors
) :
        X(std::move(iX)),
        v(std::move(iV)),
        q(iQ),
        L(std::move(iL)),
        I(iI.diagonal()),
        M(iM),
        epsilon(iEpsilon),
        functors(iFunctors) {
    F = Eigen::Vector3d(0, 0, 0);
    T = Eigen::Vector3d(0, 0, 0);
}

RigidBody3d::~RigidBody3d() =default;

Eigen::Vector3d &RigidBody3d::get_X() {
    return X;
}

Eigen::Matrix3d RigidBody3d::get_R() {
    return q.toRotationMatrix();
}

Eigen::Quaterniond &RigidBody3d::get_q() {
    return q;
}

Eigen::Vector3d &RigidBody3d::get_v() {
    return v;
}

Eigen::Vector3d &RigidBody3d::get_L() {
    return L;
}

double &RigidBody3d::get_M() {
    return M;
}

Eigen::Vector3d &RigidBody3d::get_I() {
    return I;
}

void RigidBody3d::set_inertia(Eigen::Matrix3d iI) {
    I = iI.diagonal();
}

void RigidBody3d::set_mass(double &iM) {
    M = iM;
}

void RigidBody3d::add_force(Eigen::Vector3d &iX, Eigen::Vector3d &iF) {
    F += iF;
    T += (iX - X).cross(iF);
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