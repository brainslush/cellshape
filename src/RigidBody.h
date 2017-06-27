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
        std::pair<Eigen::Vector3d,Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                A... Args
        ) {
            return std::make_pair(Eigen::Vector3d(0, 0, 0),Eigen::Vector3d(0, 0, 0));
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
                std::set<functor *> &iFunctors
        );

        RigidBody3d(
                Eigen::Vector3d iX,
                Eigen::Vector3d iV,
                Eigen::Quaterniond iQ, // rotation in lab frame
                Eigen::Vector3d iL,
                Eigen::Matrix3d iI,
                double iM,
                double iEpsilon,
                std::set<functor *> &iFunctors
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
        void do_timeStep(double &dT, A... Args) {
            // update force and torque
            F = Eigen::Vector3d(0, 0, 0);
            T = Eigen::Vector3d(0, 0, 0);
            for (auto &it : functors) {
                F += it->calcForce(X, v, q, L, Args...);
                T += it->calcTorque(X, v, q, L, Args...);
            }
            // do simulation via verlet
            do_verlet(dT, Args...);
        };

    protected:
        Eigen::Vector3d X; // position
        Eigen::Quaterniond q; // quaternion
        //Eigen::Matrix3d I; // moment of inertia
        Eigen::Vector3d I; // moment of inertia diagonalized and inversed.
        double M; // mass
        double epsilon; // precision for rotation calculation
        std::set<functor *> &functors;
        Eigen::Vector3d v; // velocity
        Eigen::Vector3d L; // angular momentum
        Eigen::Vector3d F; // forces
        Eigen::Vector3d T; // torque

        template<typename ... A>
        Eigen::Vector3d sum_(A... Args) {
            for (auto &it : functors) {

            }
        }

        template<typename ... A>
        void do_verlet(double &dT, A... Args) {
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
            Eigen::Vector3d Lbt2 = Lb + 0.5 * dT * (Tb - (Ib * Lb).cross(Lb)); // angular momentum in body frame after half time step
            Eigen::Matrix3d qkt2 = q + 0.25 * dT * R * (Ib * Lbt2); // quaternion at half time step at iteration k = 0
            Eigen::Vector3d lwt2 = L + 0.5 * dT * T; // angular momentum in lab frame
            Eigen::Matrix3d qk1t2, qdk1t2; // initialize variables for loop
            while ((qk1t2 - qkt2).norm () < epsilon) {
                Eigen::Vector3d lbt2k1 = qkt2.conjugate () *  lwt2 * qkt2; //
                Eigen::Vector3d wk1t2 = Ib * lbt2k1;
                qdk1t2 = 0.5 * qkt2 * wk1t2;
                qk1t2 = R + 0.5 * dT * qdk1t2;
            };
            q = Eigen::Quaterniond (R + dT * qdk1t2);
            Eigen::Vector3d ldt = L + dT * T;
            ldt = L + 0.5 * dT  * (T + torque.calc(xdt,vdtt,q,ldt)); // estiamte ldt
            // translation part 2
            Eigen::Vector3d adt = forceFunctor.calc(xdt,vdtt,q,ldt);
            v = v + 0.5 * dT * (a + adt);
            // rotation part 2
            Eigen::Vector3d Tdt = torqueFunctor.calc(xdt, vdtt, q, ldt);
            L = L + 0.5 * dT * (T + Tdt);
        }
    };
};

#endif /* SRC_RIGIDBODY_H_ */
