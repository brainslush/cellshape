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
        activated(iGroup->register_setting<bool>("Active", true, true)) {

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


membraneSpring::membraneSpring(mygui::group *&iGroup) :
        activated(iGroup->register_setting<bool>("Active", true, true)) {

}

membraneSpring::~membraneSpring() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
membraneSpring::calc(
        const Eigen::Vector3d &X,
        const Eigen::Vector3d &v,
        const Eigen::Quaterniond &R,
        const Eigen::Vector3d &L,
        membrane_part_base *membrane
) {
    std::pair<membrane_part_base *, membrane_part_base *> &neighbours = membrane->get_neighbours();
    double stretchForce = kStretch * (membrane->get_restLength() - membrane->get_length());
    double stretchForceNeighA = kStretch * (neighbours.first->get_restLength() - neighbours.first->get_length());
    double stretchForceNeighB =
    return pair<Eigen::Vector3d, Eigen::Vector3d>();
}


