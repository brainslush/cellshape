/*
 * The functors are classes which are handed to the rigid body class of the cell elements.
 * As subclasses of the physic::functor class they calculate interactive forces and torques of the rigid bodies.
 */

#pragma once

#include "utility"
#include "RigidBody.h"
#include "gui.h"
#include "cBaseclasses.h"

#ifndef SRC_CFUNCTORS_H_
#define SRC_CFUNCTORS_H_

namespace functor {

    /*
     * This functor calculates the friction force between Actin filaments which can be direction dependent
     */

    class ffFriction : public physic::functor {
    public:
        explicit ffFriction(mygui::gui *&iGui);

        virtual ~ffFriction();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody,
                filament_base &filament
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
        double &frictionCoeff;
        double &angleCoeff;
    };

    /*
     * This functor models the membrane - filament interaction in form of inelastic collisions.
     */

    class fmCollision : public physic::functor {
    public:
        explicit fmCollision(mygui::gui *&iGui);

        virtual ~fmCollision();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody,
                filament_base &filament
        );
    protected:
        mygui::group *guiGroup;
        bool &activated;
    };

    /*
     * Introduces a damping force to reduce momentum effects in form of viscous force
     */

    class fViscosity : public physic::functor {
    public:
        explicit fViscosity(mygui::gui *&iGui);

        virtual ~fViscosity();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
        double &factor;
    };

    /* Calculates the spring froces which model the membrane behavior */

    class membraneSpring : public physic::functor {
    public:
        membraneSpring(mygui::gui *&iGui);

        virtual ~membraneSpring();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L,
                physic::RigidBody3d &rigidBody,
                membrane_part_base &membrane
        );

    protected:
        mygui::group *guiGroup;
        bool &activated;
        double &kStretch;
        double &kBend;
    };
};

#endif // SRC_CFUNCTORS_H_
