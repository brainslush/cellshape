#include "RigidBody.h"
/*
 * RigidBody.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#include "RigidBody.h"

RigidBody::RigidBody(
  Eigen::Vector3d iX,
  Eigen::Vector3d iR, // rotation in lab frame
  Eigen::Matrix3d iI,
  double iM
):
  X(iX),
  R(iR),
  I(iI),
  M(iM)
{
  P = Eigen::Vector3d(0,0,0);
  v = Eigen::Vecotr3d(0,0,0);
  L = Eigen::Vector3d(0,0,0);
  F = Eigen::Vector3d(0,0,0);
  a = Eigen::Vector3d(0,0,0);
  T = Eigen::Vector3d(0,0,0);
  q = Eigen::Quaternion(R,0,0,1)
}

RigidBody::~RigidBody() {
  // TODO Auto-generated destructor stub
}

Eigen::Vector3d& RigidBody::get_position() {
  return X;
}
double& RigidBody::get_rotation() {
  return R;
}
void RigidBody::add_force(Eigen::Vector3d& iX, Eigen::Vector3d& iF) {
  a += iF / M;
  Eigen::Vector3d d = iX - X;
  T += d.length() * d.perpendicular().normalize().dot(iF);
}
void RigidBody::make_timeStep() {
  // calc displacement
  ;
  // calc rotation

  // reset forces and torque
  F = 0;
  T = 0;
}

void RigidBody::do_verlet (double& dT) {
  // translation
  Eigen::Vector3d xdt = X + dT * v + 0.5 * dT * dT * a;
  Eigen::Vector3d adt = ;
  Eigen::Vector3d vdt = v + 0.5 * dT * ();
  adt = ;
  v = v + 0.5 * dT * (a * adt);
  // rotation
  Eigen::Quaternion qn = q.normalize();
  Eigen::Quaternion qc = qn.conjugate();
  Eigen::Vector3d lb = qc * L * q;

  // set new position
  X = xdt;
}
