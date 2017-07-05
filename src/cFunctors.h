//
// Created by brainslush on 19.06.17.
//

#pragma once

#include "utility"
#include "RigidBody.h"
#include "gui.h"
#include "cBaseclasses.h"

#ifndef SRC_CFUNCTORS_H_
#define SRC_CFUNCTORS_H_

namespace functor {

    class filamentCollision : public physic::functor {
    public:
        filamentCollision(mygui::group *&iGroup);

        virtual ~filamentCollision();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                const Eigen::Vector3d &X,
                const Eigen::Vector3d &v,
                const Eigen::Quaterniond &R,
                const Eigen::Vector3d &L
        );

    protected:
        bool &activated;
    };

    class dampening : public physic::functor {
    public:
        dampening(mygui::group *&iGroup);

        virtual ~dampening();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                const Eigen::Vector3d &X,
                const Eigen::Vector3d &v,
                const Eigen::Quaterniond &R,
                const Eigen::Vector3d &L
        );

    protected:
        bool &activated;
    };

    class membraneSpring : public physic::functor {
    public:
        membraneSpring(mygui::group *&iGroup);

        virtual ~membraneSpring();

        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> calc(
                const Eigen::Vector3d &X,
                const Eigen::Vector3d &v,
                const Eigen::Quaterniond &R,
                const Eigen::Vector3d &L,
                membrane_part_base *cell
        );
    protected:
        bool &activated;
        double &kStretch;
        double &kBend;
    };
};

#endif // SRC_CFUNCTORS_H_
