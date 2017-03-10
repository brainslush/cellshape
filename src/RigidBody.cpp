/*
 * RigidBody.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#include "RigidBody.h"

RigidBody::RigidBody(
  ofVec2d iX,
  double iR,
  double iI,
  double iM
):
  X(iX),
  R(iR),
  I(iI),
  M(iM)
{
  P = ofVec2d(0,0);
  L = 0;
  F = ofVec2d(0,0);
  T = ofVec2d(0,0);
}

RigidBody::~RigidBody() {
  // TODO Auto-generated destructor stub
}

ofVec2d& RigidBody::get_position() {
  return X;
}
double& RigidBody::get_rotation() {
  return R;
}
void RigidBody::add_force(ofVec2d& iX, ofVec2d& iF) {
  F += iF;
  ofVec2d d = iX - X;
  T += d.length() * d.perpendicular().normalize().dot(iF);
}
void RigidBody::add_force(double iX, double& iF) {
  F += iF;
  T += iX * iF;
}
void RigidBody::make_timeStep() {
  // calc displacement
  ofVec2d a = F / M;
  // calc rotation

  // reset forces and torque
  F.x, F.y = 0;
  T = 0;
}
