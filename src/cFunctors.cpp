//
// Created by brainslush on 19.06.17.
//

#include "cFunctors.h"

using namespace functor;

filamentCollision::filamentCollision(mygui::group *&iGroup) :
        activated(iGroup->register_setting<bool>("Active", true, true)) {

}

functor::filamentCollision::~filamentCollision() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
functor::filamentCollision::calc(
        const Eigen::Vector3d &X,
        const Eigen::Vector3d &v,
        const Eigen::Quaterniond &R,
        const Eigen::Vector3d &L
) {
    if (activated) {
        return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }
    return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
}

dampening::dampening(mygui::group *&iGroup) :
        activated(iGroup->register_setting("Active", true, true)) {

}

dampening::~dampening() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
dampening::calc(
        const Eigen::Vector3d &X,
        const Eigen::Vector3d &v,
        const Eigen::Quaterniond &R,
        const Eigen::Vector3d &L
) {
    return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
}
