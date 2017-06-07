#include "RigidBody.h"

/*
 * RigidBody.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

using namespace physic;

functor::functor() {

}

functor::~functor() {

}

Eigen::Vector3d functor::calc(
        Eigen::Vector3d &X,
        Eigen::Vector3d &v,
        Eigen::Quaterniond &R,
        Eigen::Vector3d &L
) {
    return Eigen::Vector3d(0, 0, 0);
}

RigidBody3d::RigidBody3d(
        Eigen::Vector3d iX,
        Eigen::Quaterniond iQ, // rotation in lab frame
        Eigen::Matrix3d iI,
        double iM,
        double iEpsilon,
        functor *iForceFunctor,
        functor *iTorqueFunctor
) :
        X(iX),
        q(iQ),
        I(iI),
        M(iM),
        epsilon(iEpsilon),
        forceFunctor(iForceFunctor),
        torqueFunctor(iTorqueFunctor) {
    v = Eigen::Vector3d(0, 0, 0);
    L = Eigen::Vector3d(0, 0, 0);
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

void RigidBody3d::set_inertia(Eigen::Matrix3d &iI) {
    I = iI;
}

void RigidBody3d::set_mass(double &iM) {
    M = iM;
}

void RigidBody3d::add_force(Eigen::Vector3d &iX, Eigen::Vector3d &iF) {
    F += iF;
    T += (iX - X).cross(iF);
}

void RigidBody3d::do_timeStep(double &dT) {
    // update force and torque
    F = forceFunctor->calc(X, v, q, L);
    T = torqueFunctor->calc(X, v, q, L);
    // do simulation via verlet
    do_verlet(dT);
}

void RigidBody3d::do_verlet(double &dT) {
    Eigen::Vector3d a = F / M;
    // translation part 1
    Eigen::Vector3d xdt = X + dT * v + 0.5 * dT * dT * a;
    Eigen::Vector3d vdtt = v + dT * a; // something is missing here
    // rotational part 1
    q.normalize();
    Eigen::Matrix3d qm = q.matrix(); // transforming quaterinon to matrix increases speed
    Eigen::Matrix3d qc = qm.conjugate(); // conjugate of the quaternion matrix form
    /*Eigen::Vector3d lb = qc * L * qm; // angular momentum in body frame
    Eigen::Vector3d tb = (qc * T) * qm; // torque in body frame
    Eigen::Matrix3d ib = (qc * I) * qm.diagonal().inverse(); // moment of inertia in body frame diagonalized and inversed
    Eigen::Vector3d lbt2 = lb + 0.5 * dT * (tb - (ib * lb).cross(lb)); // angular momentum in body frame after half time step
    Eigen::Matrix3d qkt2 = qm + 0.25 * dT * qm * (ib * lbt2); // quaternion at half time step at iteration k = 0
    Eigen::Vector3d lwt2 = L + 0.5 * dT * T; // angular momentum in lab frame
    Eigen::Matrix3d qk1t2, qdk1t2; // initialize variables for loop
    while ((qk1t2 - qkt2).norm () < epsilon) {
        Eigen::Vector3d lbt2k1 = qkt2.conjugate () *  lwt2 * qkt2; //
        Eigen::Vector3d wk1t2 = ib * lbt2k1;
        qdk1t2 = 0.5 * qkt2 * wk1t2;
        qk1t2 = qm + 0.5 * dT * qdk1t2;
    };
    q = Eigen::Quaterniond (qm + dT * qdk1t2);
    Eigen::Vector3d ldt = L + dT * T;
    ldt = L + 0.5 * dT  * (T + torqueFunctor.calc(xdt,vdtt,q,ldt)); // estiamte ldt
    // translation part 2
    Eigen::Vector3d adt = forceFunctor.calc(xdt,vdtt,q,ldt);
    v = v + 0.5 * dT * (a + adt);
    // rotation part 2
    Eigen::Vector3d Tdt = torqueFunctor.calc(xdt, vdtt, q, ldt);
    L = L + 0.5 * dT * (T + Tdt);
    */
}
