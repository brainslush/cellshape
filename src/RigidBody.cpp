#include "RigidBody.h"

/*
 * RigidBody.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#include "RigidBody.h"

RigidBody3d::RigidBody3d (
    Eigen::Vector3d iX,
    Eigen::Vector3d iR, // rotation in lab frame
    Eigen::Matrix3d iI,
    double iM,
    Eigen::Vector3d& (*iCalcForce)(
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Vector3d& R,
        Eigen::Vector3d& L
    ),
    Eigen::Vector3d& (*iCalcTorque)(
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Vector3d& R,
        Eigen::Vector3d& L
    )
):
    X(iX),
    I(iI),
    M(iM),
    calcForce(iCalcForce),
    calcTorque(iCalcTorque)
{
    v = Eigen::Vector3d(0,0,0);
    L = Eigen::Vector3d(0,0,0);
    F = Eigen::Vector3d(0,0,0);
    T = Eigen::Vector3d(0,0,0);
    q = Eigen::Quaterniond(0, 0, 0, 1);
}
RigidBody3d::~RigidBody3d() {
    // TODO Auto-generated destructor stub
}
Eigen::Vector3d& RigidBody3d::get_position() {
    return X;
}
Eigen::Quaterniond& RigidBody3d::get_quaternion() {
    return q;
}
void RigidBody3d::add_force(Eigen::Vector3d& iX, Eigen::Vector3d& iF) {
    F += iF;
    T += (iX - X).cross(iF);
}
void RigidBody3d::set_inertia (Eigen::Matrix3d & iI) {
    I = iI;
}
void RigidBody3d::make_timeStep(double& dT) {
    // reset forces and torque
    F = Eigen::Vector3d(0,0,0);
    T = Eigen::Vector3d(0,0,0);
}

void RigidBody3d::do_verlet (double& dT) {
    Eigen::Vector3d a = F / M;
    // translation part 1
    Eigen::Vector3d xdt = X + dT * v + 0.5 * dT * dT * a;
    Eigen::Vector3d vdtt = v + dT * a;
    // rotational part 1
    q.normalize();
    Eigen::Matrix3d qm = q.matrix(); // transforming quaterion to matrix increases speed
    Eigen::Matrix3d qc = qm.conjugate(); // conjugate of the quaternion matrix form
    Eigen::Vector3d lb = qc * L * qm; // angular momentum in body frame
    Eigen::Vector3d tb = qc * T * qm; // torque in body frame
    Eigen::Matrix3d ib = (qc * I * qm).diagonal().inverse(); // moment of inertia in body frame diagonalized and inversed
    Eigen::Vector3d lbt2 = lb + 0.5 * dT * (tb - (ib * lb).cross(lb)); // angular momentum in body frame after half time step
    Eigen::Matrix3d qkt2 = qm + 0.25 * dT * qm * (ib * lbt2); // quaternion at half time step at iteration k = 0
    Eigen::Vector3d lwt2 = L + 0.5 * dT * T; // angular momentum in lab frame
    Eigen::Matrix3d qk1t2, qdk1t2;
    while ((qk1t2 - qkt2).norm () < EPSILON) {
        Eigen::Vector3d lbt2k1 = qkt2.conjugate () *  lwt2 * qkt2; // 
        Eigen::Vector3d wk1t2 = ib * lbt2;
        qdk1t2 = 0.5 * qkt2 * wk1t2;
        qk1t2 = qm + 0.5 * dT * qdk1t2;
    };
    Eigen::Quaterniond qdt = Eigen::Quaterniond (qm + dT * qdk1t2);
    Eigen::Vector3d ldt = L + dT * T;
    ldt = L + 0.5 * dT  * (T + calcTorque(xdt,vdtt,qdt,ldt)); // estiamte ldt
    // calc new forces and torques
    Eigen::Vector3d adt = calcForce(xdt,vdtt,qdt,ldt);
    Eigen::Vector3d Tdt = calcTorque (xdt, vdtt, qdt, ldt);
    // translation part 2
    Eigen::Vector3d vdt = v + 0.5 * dT * (a + adt);
    Eigen::Vector3d ldt
    // rotation
    

    // set new position
    X = xdt;
}

RigidBody2d::RigidBody2d (
    Eigen::Vector2d iX,
    double iR,
    double iI,
    double iM,
    Eigen::Vector2d (*iCalcForce)(
    Eigen::Vector2d& X,
    Eigen::Vector2d& v,
    double& R,
    double& L
),
double (*iCalcTorque)(
    Eigen::Vector2d& X,
    Eigen::Vector2d& v,
    double& R,
    double& L
)
) :
    X (iX),
    I (iI),
    M (iM),
    R (iR),
    calcForce (iCalcForce),
    calcTorque (iCalcTorque)
{
    v = Eigen::Vector2d (0, 0);
    L = 0;
    F = Eigen::Vector2d (0, 0);
    T = 0;
    q = Eigen::Quaterniond (0, 0, 0, 1);
};
RigidBody2d::~RigidBody2d () {

};
Eigen::Vector2d& RigidBody2d::get_position () {
    return X;
};
double RigidBody2d::get_rotation () {
    return R
};
void RigidBody2d::add_force (Eigen::Vector2d& iX, Eigen::Vector2d& iF) {
    F += iF;
    Eigen::Vector2d dX = iX - X;
    T += dX[0] * iF[1] - dX[1] * iF[0];
};
void RigidBody2d::set_inertia (double& iI) {
    I = iI;
};
void RigidBody2d::make_timeStep (double& dT) {

};
void RigidBody2d::do_verlet (double& dT) {
    Eigen::Vector2d a = F / M;
    // translation part 1
    Eigen::Vector2d xdt = X + dT * v + 0.5 * dT * dT * a;
    Eigen::Vector2d vdtt = v + dT * a;
    // rotational part 1
    double lt2 = L + 0.5 * dT * T;
    double r0t2 = 0.5 * R + 0.25 * dT * R * lt2 / I;
    double rk1;
    while (abs (rk1 - rk) < EPSILON) {
        double w = lt2 / I;
        rk1 = r + 0.5 * w;
    };
    // 
    // calc new forces and torques
    Eigen::Vector3d adt = calcForce (xdt, vdtt, qdt, ldt);
    Eigen::Vector3d Tdt = calcTorque (xdt, vdtt, qdt, ldt)
        // translation part 2
        Eigen::Vector3d vdt = v + 0.5 * dT;
    v = v + 0.5 * dT * (a * adt);
    // rotation


    // set new position
    X = xdt;
};