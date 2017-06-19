/*
 * RigidBody.h
 *
 *  Created on: Mar 8, 2017
 *      Author: siegbahn
 */

#pragma once

#include <iostream>
#include <set>
#include <eigen3/Eigen/Eigen>

#ifndef SRC_RIGIDBODY_H_
#define SRC_RIGIDBODY_H_

namespace physic {
    /********************************************/
    /* Functor for torque and force calculation */
    class functor {
    public:
        functor() {

        };

        virtual ~functor() {

        };

        template<typename ... A>
        virtual Eigen::Vector3d calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                A &Args
        ) {
            return Eigen::Vector3d(0, 0, 0);
        };
    };

    /*******************************************************************/
    /* Rigid body class which uses velocity-verlet to calculate changes*/
    class RigidBody3d {
    public:
        RigidBody3d(
                Eigen::Vector3d iX,
                Eigen::Quaterniond iQ, // rotation in lab frame
                Eigen::Matrix3d iI,
                double iM,
                double iEpsilon,
                std::set<functor *> &iForceFunctors,
                std::set<functor *> &iTorqueFunctors
        );

        RigidBody3d(
                Eigen::Vector3d iX,
                Eigen::Vector3d iV,
                Eigen::Quaterniond iQ, // rotation in lab frame
                Eigen::Vector3d iL,
                Eigen::Matrix3d iI,
                double iM,
                double iEpsilon,
                std::set<functor *> &iForceFunctors,
                std::set<functor *> &iTorqueFunctors
        );

        virtual ~RigidBody3d();

        virtual Eigen::Vector3d &get_position();

        virtual Eigen::Matrix3d get_rotationMatrix();

        virtual Eigen::Quaterniond &get_quaternion();

        virtual Eigen::Vector3d &get_velocity();

        virtual Eigen::Vector3d &get_angularMomentum();

        virtual void set_inertia(Eigen::Matrix3d iI);

        virtual void set_mass(double &iM);

        virtual void add_force(Eigen::Vector3d &iX, Eigen::Vector3d &iF);

        template<typename ... A>
        virtual void do_timeStep(double &dT, A Args) {
            // variableUpdate force and torque
            F = Eigen::Vector3d(0, 0, 0);
            for (auto &it : forceFunctors) {
                F += it->calc(X, v, q, L, Args...);
            }
            T = Eigen::Vector3d(0, 0, 0);
            for (auto &it : torqueFunctors) {
                T += it->calc(X, v, q, L, Args...);
            }
            // do simulation via verlet
            do_verlet(dT);
        };

    protected:
        Eigen::Vector3d X; // position
        Eigen::Quaterniond q; // quaternion
        //Eigen::Matrix3d I; // moment of inertia
        Eigen::Vector3d I; // moment of inertia diagonalized and inversed.
        double M; // mass
        double epsilon; // precision for rotation calculation
        std::set<functor *> &forceFunctors;
        std::set<functor *> &torqueFunctors;
        Eigen::Vector3d v; // velocity
        Eigen::Vector3d L; // angular momentum
        Eigen::Vector3d F; // forces
        Eigen::Vector3d T; // torque
        virtual void do_verlet(double &dT);
    };
};

#endif /* SRC_RIGIDBODY_H_ */
