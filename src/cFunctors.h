//
// Created by brainslush on 19.06.17.
//

#pragma once

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

        virtual std::pair<Eigen::Vector3d,Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L
        );

    protected:
        bool &activated;
    };

    class dampening : public physic::functor {
    public:
        dampening(mygui::group *&iGroup);

        virtual ~dampening();

        virtual std::pair<Eigen::Vector3d,Eigen::Vector3d> calc(
                Eigen::Vector3d &X,
                Eigen::Vector3d &v,
                Eigen::Quaterniond &R,
                Eigen::Vector3d &L
        );

    protected:
        bool &activated;
    };

};

#endif // SRC_CFUNCTORS_H_
