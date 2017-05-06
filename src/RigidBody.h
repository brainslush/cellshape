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
      Eigen::Vector3d iX,
      Eigen::Vector3d iR, // rotation in lab frame
      Eigen::Matrix3d iI,
      double iM
  );
  virtual ~RigidBody();
  virtual Eigen::Vector3d& get_position();
  virtual double& get_rotation();
  virtual void add_force(Eigen::Vector3d& iX, Eigen::Vector3d& iF);
  virtual void make_timeStep();
protected:
  Eigen::Vector3d X; // position
  Eigen::Quaterniond q; // quaterion
  //Eigen::Vector3d R; // rotation
  //Eigen::Vector3d P; // momentum
  Eigen::Vector3d v; // velocity
  Eigen::Vector3d L; // angular momentum
  Eigen::Matrix3d I; // moment of inertia
  double M; // mass
  Eigen::Vector3d a; // forces
  Eigen::Vector3d T; // torque
  void do_verlet ();
  void do_rk4 ();
};

#endif /* SRC_RIGIDBODY_H_ */
