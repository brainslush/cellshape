/*
 * Handles rigid body simulation
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

    /*
     * Functor for torque and force calculation
     * */

    class functor {
    public:

        functor() = default;

        virtual ~functor() = default;

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

    /*
     * Rigid body class which uses numerical integrators to calculate changes
     * */

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

        /*
         * Simulate single time step
         * */

        template<typename ... A>
        void do_timeStep(double &dT, A... Args) {
            // update force and torque
            auto _temp = sum_functors(X, v, q, L, *this, Args...);
            F = _temp.first;
            T = _temp.second;
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
        auto qsum(L l, R r) {
            Eigen::Quaterniond c;
            c.coeffs() = l.coeffs() + r.coeffs();
            return c;
        };

        template<typename L, typename R>
        auto qdiff(L l, R r) {
            Eigen::Quaterniond c;
            c.coeffs() = l.coeffs() - r.coeffs();
            return c;
        };

        Eigen::Quaterniond qscale(const double &s, const Eigen::Quaterniond &q);

        Eigen::Quaterniond vec2quat(const Eigen::Vector3d &v);

        /*
         * sum up both force and torque functors
         * */
        /*
        template<typename ... A>
        auto sum_functors(
                Eigen::Vector3d &iX,
                Eigen::Vector3d &iv,
                Eigen::RotationBase<double,3> &iR,
                Eigen::Vector3d &iL,
                RigidBody3d &rigidBody,
                A... Args
        ) {
            return sum_functors(
                    iX,
                    iv,
                    Eigen::Quaternion(iR),
                    iL
            );
        }*/

        template<typename ... A>
        auto sum_functors(
                Eigen::Vector3d &iX,
                Eigen::Vector3d &iv,
                Eigen::Quaterniond &iR,
                Eigen::Vector3d &iL,
                RigidBody3d &rigidBody,
                A... Args
        ) {
            std::pair < Eigen::Vector3d, Eigen::Vector3d > _ret;
            if (functors) {
                for (auto &_it : *functors) {
                    _ret += _it->calc(iX, iv, iR, iL, rigidBody, Args...);
                }
            }
            return _ret;
        }

        /*
         * sum up force functors
         */

        template<typename ... A>
        auto calc_force(
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

        /*
         * sum up torque functors
         * */

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
         * Numerical integration method based on the 4th order Runge-Kutta algorithm
         * */

        template<typename ... A>
        void do_rk4(double &dT, A... Args) {

            auto _a1 = F / M;
            // make sure q is normalized
            q.normalize();
            // get quaternion rotation as rotation matrix form
            auto _R = q.toRotationMatrix();
            // calculate inverse of
            auto _Ib = _R * I;
            _Ib = _Ib.cwiseInverse();

            // angular momentum in body frame
            auto _Lb1 = _R * L;

            // calculate first half time step
            auto _v2 = v + 0.5 * dT * _a1;
            auto _x = X + 0.5 * dT * _v2;
            auto _L = L + 0.5 * dT * T;
            auto _Lb2 = _R * _L;
            auto _q = qsum(q, q * vec2quat(_Ib.cwiseProduct(0.5 * dT * _Lb2)));

            auto _k2 = sum_functors(_x, _v2, _q, _L, *this, Args...);

            // calculate another half time step with new values
            auto _v3 = v + 0.5 * dT * _k2.first;
            _x = X + 0.5 * dT * _v3;
            _L = L + 0.5 * dT * _k2.second;
            auto _Lb3 = _R * _L;
            _q = qsum(q, q * vec2quat(_Ib.cwiseProduct(0.5 * dT * _Lb3)));

            auto _k3 = sum_functors(_x, _v3, _q, _L, *this, Args...);

            // and do it a third time with a full time step
            auto _v4 = v + dT * _k3.first;
            _x = X + dT * _v4;
            auto _Lb4 = L + dT * _k3.second;
            _q = qsum(q, q * vec2quat(_Ib.cwiseProduct(0.5 * dT * _Lb4)));

            auto _k4 = sum_functors(_x, _v4, _q, _L, *this, Args...);

            // obtain new position and velocity, roation, angular momemntum
            X = X + 1 / 6 * dT * (v + 2 * _v2 + 2 * _v3 + _v4);
            v = v + 1 / 6 * dT * (_a1 + 2 * _k2.first + 2 * _k3.first + _k4.first);
            q = qsum(q, q * vec2quat(1/6 * dT * _Ib.cwiseProduct(L + 2 * _Lb2 + 2 * _Lb3 + _Lb4)));
            L = L + 1/6 * dT * (T + 2 * _k2.second + 2 * _k3.second + _k4.second);
        }

        /*
         * Numerical integration method based on a modified version of
         * the velocity verlet algorithm
         * */

        template<typename ... A>
        void do_vverlet(double &dT, A... Args) {
            auto _a = F / M;
            // translation part 1
            X = X + dT * v + 0.5 * dT * dT * _a; // calculating new position
            // rotational part 1
            q.normalize();
            // transforming quaterinon to rotation matrix increases speed
            auto _R = q.toRotationMatrix();
            std::cout << "This is R:\n" << _R << "\n";
            std::cout << "This is L:\n" << L << "\n";
            // angular momentum in body frame
            auto _Lb = _R * L;
            std::cout << "This is L in body frame:  \n" << _Lb << "\n";
            std::cout << "This is T:\n" << _Lb << "\n";
            // torque in body frame
            auto _Tb = _R * T;
            std::cout << "This is T in body frame:  \n" << _Tb << "\n";
            // moment of inertia in body frame diagonalized and inversed
            auto _Ib = _R * I;
            _Ib = _Ib.cwiseInverse();
            std::cout << "This is I⁻¹m:\n" << _Ib << "\n";
            // angular momentum in body frame after half time step
            auto _Lbt2 = _Lb + 0.5 * dT * (_Tb - (_Ib.cwiseProduct(_Lb)).cross(_Lb));
            // quaternion at half time step at iteration k = 0
            auto _qkt2 = qsum(q, q * vec2quat(_Ib.cwiseProduct(0.25 * dT * _Lbt2)));
            // angular momentum in lab frame
            Eigen::Quaterniond _Lwt2;
            _Lwt2.w() = 0;
            _Lwt2.vec() = L + 0.5 * dT * T;
            // precision itterator
            auto _qk1t2 = _qkt2;
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
            auto _Ldt = L + dT * T;
            auto _vdt = v + dT * _a;

            _vdt = v + 0.5 * dT * (_a + calc_force(X, _vdt, q, _Ldt, *this, Args...)); // estimate vdt
            _Ldt = L + 0.5 * dT * (T + calc_torque(X, _vdt, q, _Ldt, *this, Args...)); // estimate ldt
            auto _newFT = sum_functors(X, _vdt, q, _Ldt, *this, Args...);
            v = v + 0.5 * dT * (_a + _newFT.first);
            L = L + 0.5 * dT * (T + _newFT.second);
        }
    };
};

#endif /* SRC_RIGIDBODY_H_ */
