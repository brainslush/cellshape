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
        std::set<functor*> &iForceFunctors,
        std::set<functor*> &iTorqueFunctors
) :
        X(iX),
        q(iQ),
        I(iI.inverse().diagonal()),
        M(iM),
        epsilon(iEpsilon),
        forceFunctors(iForceFunctors),
        torqueFunctors(iTorqueFunctors) {
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
        std::set<functor*> &iForceFunctors,
        std::set<functor*> &iTorqueFunctors
) :
        X(iX),
        v(iV),
        q(iQ),
        L(iL),
        I(iI.inverse().diagonal()),
        M(iM),
        epsilon(iEpsilon),
        forceFunctors(iForceFunctors),
        torqueFunctors(iTorqueFunctors) {
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

void RigidBody3d::do_verlet(double &dT) {
    Eigen::Vector3d a = F / M;
    // translation part 1
    Eigen::Vector3d xdt = X + dT * v + 0.5 * dT * dT * a;
    Eigen::Vector3d vdtt = v + dT * a; // something is missing here
    // rotational part 1
    q.normalize();
    Eigen::Matrix3d R = q.toRotationMatrix(); // transforming quaterinon to rotation matrix increases speed
    //Eigen::Matrix3d qmc = q.conjugate().toRotationMatrix(); // conjugation of the quaternion matrix form
    //std::cout << "This is q: \n" << q << "\n";
    std::cout << "This is R:\n" << R << "\n";
    std::cout << "This is L:\n" << L << "\n";
    Eigen::Vector3d Lb = R * L; // angular momentum in body frame
    std::cout << "This is L in body frame:  \n" << Lb << "\n";
    std::cout << "This is T:\n" << Lb << "\n";
    Eigen::Vector3d Tb = R * T; // torque in body frame
    std::cout << "This is T in body frame:  \n" << Tb << "\n";
    std::cout << "This is I⁻¹:\n" << I << "\n";
    Eigen::Vector3d Ib = R * I; // moment of inertia in body frame diagonalized and inversed
    std::cout << "This is I⁻¹m:\n" << Ib << "\n";
    /*Eigen::Vector3d lbt2 = lb + 0.5 * dT * (tb - (ib * lb).cross(lb)); // angular momentum in body frame after half time step
    Eigen::Matrix3d qkt2 = qm + 0.25 * dT * qm * (ib * lbt2); // quaternion at half time step at iteration k = 0
    Eigen::Vector3d lwt2 = L + 0.5 * dT * T; // angular momentum in lab frame
    Eigen::Matrix3d qk1t2, qdk1t2; // initialize variables for loop
    /*while ((qk1t2 - qkt2).norm () < epsilon) {
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
