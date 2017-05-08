/*
 * RigidBody.h
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#include <eigen3/Eigen/Eigen>

#ifndef SRC_RIGIDBODY_H_
#define SRC_RIGIDBODY_H_

class RigidBody3d {
public:
  RigidBody3d(
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
     ),
     double iEpsilon
  );
  virtual ~RigidBody3d();
  virtual Eigen::Vector3d& get_position ();
  virtual Eigen::Vector3d get_rotation ();
  virtual Eigen::Quaterniond& get_rotation ();
  virtual Eigen::Vector3d& get_velocity ();
  virtual Eigen::Vector3d& get_angularMomentum ();
  virtual void set_inertia (Eigen::Matrix3d& iI);
  virtual void set_mass (double& iM);
  virtual void add_force (Eigen::Vector3d& iX, Eigen::Vector3d& iF);
  virtual void do_timeStep (double& dT);

protected:
    Eigen::Vector3d& (*calcForce)(
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Quaterniond& R,
        Eigen::Vector3d& L
    );
    Eigen::Vector3d& (*calcTorque)(
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Quaterniond& R,
        Eigen::Vector3d& L
    );
    double epsilon;
    Eigen::Vector3d X; // position
    Eigen::Quaterniond q; // quaterion
    Eigen::Vector3d v; // velocity
    Eigen::Vector3d L; // angular momentum
    Eigen::Matrix3d I; // moment of inertia
    double M; // mass
    Eigen::Vector3d F; // forces
    Eigen::Vector3d T; // torque
    virtual void do_verlet (double& dT);
};

#endif /* SRC_RIGIDBODY_H_ */
