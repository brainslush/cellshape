/*
 * RigidBody.h
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#include "variable_type.h"

#ifndef SRC_RIGIDBODY_H_
#define SRC_RIGIDBODY_H_

#define EPSILON 0.00001

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
     )
  );
  virtual ~RigidBody3d();
  virtual Eigen::Vector3d& get_position ();
  virtual Eigen::Vector3d get_rotation ();
  virtual Eigen::Quaterniond& get_quaternion ();
  virtual void add_force (Eigen::Vector3d& iX, Eigen::Vector3d& iF);
  virtual void set_inertia (Eigen::Matrix3d& iI);
  virtual void make_timeStep (double& dT);
protected:
    Eigen::Vector3d& (*calcForce)(
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Vector3d& R,
        Eigen::Vector3d& L
    );
    Eigen::Vector3d& (*calcTorque)(
        Eigen::Vector3d& X,
        Eigen::Vector3d& v,
        Eigen::Vector3d& R,
        Eigen::Vector3d& L
    ];
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
class RigidBody2d {
public:
    RigidBody2d (
        Eigen::Vector2d iX,
        double iR, // rotation in lab frame
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
    );
    virtual ~RigidBody2d ();
    virtual Eigen::Vector2d& get_position ();
    virtual double get_rotation ();
    virtual double R ();
    virtual void add_force (Eigen::Vector2d& iX, Eigen::Vector2d& iF);
    virtual void set_inertia (double& iI);
    virtual void make_timeStep (double& dT);
protected:
    Eigen::Vector2d (*calcForce)(
        Eigen::Vector2d& X,
        Eigen::Vector2d& v,
        double& R,
        double& L
        );
    double (*calcTorque)(
        Eigen::Vector2d& X,
        Eigen::Vector2d& v,
        double& R,
        double& L
    ];
    Eigen::Vector2d X; // position
    double R; // rotation
    Eigen::Vector2d v; // velocity
    double L; // angular momentum
    double I; // moment of inertia
    double M; // mass
    Eigen::Vector2d F; // forces
    double T; // torque
    virtual void do_verlet (double& dT);
};

#endif /* SRC_RIGIDBODY_H_ */
