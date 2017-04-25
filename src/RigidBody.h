/*
 * RigidBody.h
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#include "variable_type.h"

#ifndef SRC_RIGIDBODY_H_
#define SRC_RIGIDBODY_H_

class RigidBody {
public:
  RigidBody(
      ofVec2d iX,
      double iR,
      double iI,
      double iM
  );
  virtual ~RigidBody();
  virtual ofVec2d& get_position();
  virtual double& get_rotation();
  virtual void add_force(ofVec2d& iX, ofVec2d& iF);
  virtual void add_force(double iX, double& iF);
  virtual void make_timeStep();
protected:
  ofVec2d X; // position
  double R; // orientation
  ofVec2d P; // momentum
  double L; // angular momentum
  double I; // moment of inertia
  double M; // mass
  ofVec2d F; // forces
  double T; // torque
  void do_verlet ();
  void do_rk4 ();
};

#endif /* SRC_RIGIDBODY_H_ */
