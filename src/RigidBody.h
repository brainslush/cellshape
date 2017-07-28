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
#include "std.h"

#ifndef SRC_RIGIDBODY_H_
#define SRC_RIGIDBODY_H_

namespace physic {

    // ahead declaration
    class RigidBody3d;

    // Functor for torque and force calculation
    class functor {
    public:

        functor() {

        };

        virtual ~functor() {

        };

        template<typename ... A>
        std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                RigidBody3d &rigidBody3d,
                A... Args
        ) {
            return {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
        };

        template<typename ... A>
        Eigen::Vector3d calc_force(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                RigidBody3d &rigidBody3d,
                A... Args
        ) {
            return Eigen::Vector3d(0, 0, 0);
        };

        template<typename ... A>
        Eigen::Vector3d calc_torque(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                RigidBody3d &rigidBody3d,
                A... Args
        ) {
            return Eigen::Vector3d(0, 0, 0);
        };
    };

    // Rigid body class which uses numerical integrators to calculate changes
    class RigidBody3d {

    public:

        RigidBody3d();

        RigidBody3d(
                Eigen::Vector3d iX,
                Eigen::Quaterniond iQ, // rotation in lab frame
                Eigen::Matrix3d iI,
                double iM,
                double iEpsilon,
                std::set<functor *> *iFunctors
        );

        RigidBody3d(
                Eigen::Vector3d iX,
                Eigen::Vector3d iV,
                Eigen::Quaterniond iQ, // rotation in lab frame
                Eigen::Vector3d iL,
                Eigen::Matrix3d iI,
                double iM,
                double iEpsilon,
                std::set<functor *> *iFunctors
        );

        virtual ~RigidBody3d();

        virtual Eigen::Vector3d &get_X();

        virtual Eigen::Matrix3d get_R();

        virtual Eigen::Quaterniond &get_q();

        virtual Eigen::Vector3d &get_v();

        virtual Eigen::Vector3d &get_L();

        virtual double &get_M();

        virtual Eigen::Vector3d &get_I();

        virtual void set_inertia(Eigen::Matrix3d iI);

        virtual void set_mass(double &iM);

        virtual void add_force(Eigen::Vector3d &iX, Eigen::Vector3d &iF);

        template<typename ... A>
        void do_timeStep(double &dT, A... Args) {
            // update force and torque
            std::pair<Eigen::Vector3d, Eigen::Vector3d> temp = sum_functors(X, v, q, L, *this, Args...);
            F = temp.first;
            T = temp.second;
            // do simulation via verlet
            do_vverlet(dT, Args...);
        }

    protected:
        Eigen::Vector3d X; // position
        Eigen::Quaterniond q; // quaternion
        Eigen::Vector3d I; // moment of inertia diagonalized.
        double M; // mass
        double epsilon; // precision for rotation calculation
        std::set<functor *> *functors; // functors which calculate forces and torques
        Eigen::Vector3d v; // velocity
        Eigen::Vector3d L; // angular momentum
        Eigen::Vector3d F; // forces
        Eigen::Vector3d T; // torque



        template<typename L, typename R>
        Eigen::Quaterniond qsum(L l, R r) {
            Eigen::Quaterniond c;
            c.coeffs() = l.coeffs() + r.coeffs();
            return c;
        };

        template<typename L, typename R>
        Eigen::Quaterniond qdiff(L l, R r) {
            Eigen::Quaterniond c;
            c.coeffs() = l.coeffs() - r.coeffs();
            return c;
        };

        Eigen::Quaterniond qscale(const double &s, const Eigen::Quaterniond &q);

        Eigen::Quaterniond qvecprod(const Eigen::Vector3d &v, const Eigen::Quaterniond &q);

        Eigen::Quaterniond vec2quat(const Eigen::Vector3d &v);

        template<typename ... A>
        std::pair<Eigen::Vector3d, Eigen::Vector3d> sum_functors(
                Eigen::Vector3d &iX,
                Eigen::Vector3d &iv,
                Eigen::Quaterniond &iR,
                Eigen::Vector3d &iL,
                RigidBody3d &rigidBody,
                A... Args
        ) {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> ret;
            if (functors) {
                for (auto &it : *functors) {
                    ret += it->calc(iX, iv, iR, iL, rigidBody, Args...);
                }
            }
            return ret;
        }

        template<typename ... A>
        Eigen::Vector3d calc_force(
                Eigen::Vector3d &iX,
                Eigen::Vector3d &iv,
                Eigen::Quaterniond &iR,
                Eigen::Vector3d &iL,
                RigidBody3d &rigidBody,
                A... Args
        ) {
            Eigen::Vector3d ret;
            if (functors) {
                for (auto &it : *functors) {
                    ret += it->calc_force(iX, iv, iR, iL, rigidBody, Args...);
                }
            }
            return ret;
        }

        template<typename ... A>
        Eigen::Vector3d calc_torque(
                Eigen::Vector3d &iX,
                Eigen::Vector3d &iv,
                Eigen::Quaterniond &iR,
                Eigen::Vector3d &iL,
                RigidBody3d &rigidBody,
                A... Args
        ) {
            Eigen::Vector3d ret;
            if (functors) {
                for (auto &it : *functors) {
                    ret += it->calc_torque(iX, iv, iR, iL, rigidBody, Args...);
                }
            }
            return ret;
        }

        template<typename ... A>
        void do_leapFrog(double &dT, A... Args) {
            Eigen::Vector3d _a = F / M;
            Eigen::Vector3d _X = X + dT * v + 0.5 * dT * dT * _a;
            //Eigen::Vector3d _v =
        }

        /*
        template<typename ... A>
        void do_rk4(double &dT, A... Args) {
            Eigen::Vector3d k1 = F / M;
            Eigen::Vector3d _x = X + 0.5 * dT * v;
            Eigen::Quaterniond =
            Eigen::Vector3d k2 = calc_force(_x + 0.5 * dT * dT,)
            Eigen::Vector3d k3 = calc_force()
            //
        }
         */

        template<typename ... A>
        void do_vverlet(double &dT, A... Args) {
            Eigen::Vector3d _a = F / M;
            // translation part 1
            X = X + dT * v + 0.5 * dT * dT * _a; // calculating new position
            // rotational part 1
            // normalize q
            q.normalize();
            // transforming quaterinon to rotation matrix increases speed
            Eigen::Matrix3d _R = q.toRotationMatrix();
            std::cout << "This is R:\n" << _R << "\n";
            std::cout << "This is L:\n" << L << "\n";
            // angular momentum in body frame
            Eigen::Vector3d _Lb = _R * L;
            std::cout << "This is L in body frame:  \n" << _Lb << "\n";
            std::cout << "This is T:\n" << _Lb << "\n";
            // torque in body frame
            Eigen::Vector3d _Tb = _R * T;
            std::cout << "This is T in body frame:  \n" << _Tb << "\n";
            // moment of inertia in body frame diagonalized and inversed
            Eigen::Vector3d _Ib = _R * I;
            _Ib = _Ib.cwiseInverse();
            std::cout << "This is I⁻¹m:\n" << _Ib << "\n";
            // angular momentum in body frame after half time step
            Eigen::Vector3d _Lbt2 = _Lb + 0.5 * dT * (_Tb - (_Ib.cwiseProduct(_Lb)).cross(_Lb));
            // quaternion at half time step at iteration k = 0
            Eigen::Quaterniond _qkt2 = qsum(q, q * vec2quat(_Ib.cwiseProduct(0.25 * dT * _Lbt2)));
            // angular momentum in lab frame
            Eigen::Quaterniond _Lwt2;
            _Lwt2.w() = 0;
            _Lwt2.vec() = L + 0.5 * dT * T;
            // precision itterator
            Eigen::Quaterniond _qk1t2 = _qkt2;
            Eigen::Quaterniond _qdk1t2;
            Eigen::Vector3d _Lbk1t2;
            do {
                _qkt2 = _qk1t2;
                _Lbk1t2 = (_qkt2 * _Lwt2 * _qkt2.inverse()).vec();
                _qdk1t2 = qscale(0.5, _qkt2) * vec2quat(_Ib.cwiseProduct(_Lbk1t2));
                _qk1t2 = qsum(q, qscale(0.5 * dT, _qdk1t2));
            } while ((qdiff(_qk1t2, _qkt2)).norm() > epsilon);
            q = qsum(q, qscale(dT, _qdk1t2));
            // part 2 estimate L and v w/ an estimated T and a at t+dt
            Eigen::Vector3d _Ldt = L + dT * T;
            Eigen::Vector3d _vdt = v + dT * _a;

            _vdt = v + 0.5 * dT * (_a + calc_force(X, _vdt, q, _Ldt, *this, Args...)); // estimate vdt
            _Ldt = L + 0.5 * dT * (T + calc_torque(X, _vdt, q, _Ldt, *this, Args...)); // estimate ldt
            std::pair<Eigen::Vector3d, Eigen::Vector3d> newFT = sum_functors(X, _vdt, q, _Ldt, *this, Args...);
            v = v + 0.5 * dT * (_a + newFT.first);
            L = L + 0.5 * dT * (T + newFT.second);
        }
    };
};

#endif /* SRC_RIGIDBODY_H_ */
