//
// Created by brainslush on 19.06.17.
//

#include "cFunctors.h"

using namespace functor;

ffFriction::ffFriction(mygui::gui *&iGui) :
        functor(),
        guiGroup(iGui->register_group("F-F Friction")),
        activated(guiGroup->register_setting<bool>("Active", true, true)),
        frictionCoeff(guiGroup->register_setting<double>("Friction Coefficient", true, 0, 1, 1)),
        angleCoeff(guiGroup->register_setting<double>("Angle Coefficient", true, 0, 1, 1)) {

}

functor::ffFriction::~ffFriction() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ffFriction::calc(
        Eigen::Vector3d &X,
        Eigen::Vector3d &v,
        Eigen::Quaterniond &R,
        Eigen::Vector3d &L,
        physic::RigidBody3d &rigidBody,
        filament_base &filament
) {
    if (activated) {
        auto &intersectors = filament.get_intersectors();
        auto &positions = filament.get_positions();
        Eigen::Vector3d com = 0.5 * (positions[0] + positions[1]);
        Eigen::Vector3d force(0, 0, 0);
        Eigen::Vector3d torque(0, 0, 0);
        // run through all elements which intersect this filament
        for (auto &it : intersectors) {

            auto &el = it.first;
            auto &pos = it.second;
            // check if element is acutally a filament
            if (filament_base *element = dynamic_cast<filament_base *>(el)) {
                Eigen::Vector3d vNorm = element->get_rigidBody().get_v().normalized();
                force += frictionCoeff * vNorm;
                torque += frictionCoeff * (pos - com).cross(vNorm);
            }
        }
        return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }
    return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
}

fmCollision::fmCollision(mygui::gui *&iGui) :
        functor(),
        guiGroup(iGui->register_group("F-M Collision")),
        activated(guiGroup->register_setting<bool>("Active", true, true)) {

}

fmCollision::~fmCollision() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
fmCollision::calc(Eigen::Vector3d &X, Eigen::Vector3d &v, Eigen::Quaterniond &R, Eigen::Vector3d &L,
                  physic::RigidBody3d &rigidBody, filament_base &filament) {
    return pair<Eigen::Vector3d, Eigen::Vector3d>();
}

fViscosity::fViscosity(mygui::gui *&iGui) :
        guiGroup(iGui->register_group("Dampening")),
        activated(guiGroup->register_setting<bool>("Active", true, true)),
        factor(guiGroup->register_setting<double>("Factor", true, 0, 1, 0.1)) {
}

fViscosity::~fViscosity() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
fViscosity::calc(
        Eigen::Vector3d &X,
        Eigen::Vector3d &v,
        Eigen::Quaterniond &R,
        Eigen::Vector3d &L,
        physic::RigidBody3d &rigidBody
) {
    if (activated) {
        Eigen::Vector3d _F = -factor * rigidBody.get_M * v;
        Eigen::Vector3d _T = -factor * rigidBody.get_I().cwiseInverse().cwiseProduct(L);
        return {_F, _T};
    } else {
        return {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
    }
}


membraneSpring::membraneSpring(mygui::gui *&iGui) :
        guiGroup(iGui->register_group("Spring Force")),
        activated(guiGroup->register_setting<bool>("Active", true, true)),
        kStretch(guiGroup->register_setting<double>("kStretch", true, 0, 1, 0.1)),
        kBend(guiGroup->register_setting<double>("kBend", true, 0, 1, 0.1)) {

}

membraneSpring::~membraneSpring() {

}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
membraneSpring::calc(
        Eigen::Vector3d &X,
        Eigen::Vector3d &v,
        Eigen::Quaterniond &R,
        Eigen::Vector3d &L,
        physic::RigidBody3d &rigidBody,
        membrane_part_base *membrane
) {
    if (activated) {
        // acquire some data
        std::pair<membrane_part_base *, membrane_part_base *> &neighbours = membrane->get_neighbours();
        std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &sharedPositions = membrane->get_sharedPositions();
        double length = membrane->get_length();
        // calculate the normalized direction vector of the membrane element
        Eigen::Vector3d dirVector = membrane->calc_dirVector(&membrane->get_positions()[1]);
        // calculate the normalized direction vectors of the neighbouring membrane elements
        Eigen::Vector3d springForceNeighA = neighbours.first->calc_dirVector(sharedPositions.first);
        Eigen::Vector3d springForceNeighB = neighbours.second->calc_dirVector(sharedPositions.second);
        // calculate the torsion force between the membrane and the neighbours
        Eigen::Vector3d springTorqueNeighA = kBend * springForceNeighA.cross(dirVector);
        Eigen::Vector3d springTorqueNeighB = kBend * springForceNeighB.cross(-dirVector);
        // calculate the stretching force generated by the neighbouring membrane elements
        springForceNeighA *= -kStretch * (neighbours.first->get_restLength() - neighbours.first->get_length());
        springForceNeighB *= -kStretch * (neighbours.second->get_restLength() - neighbours.second->get_length());
        // calculate torque due to stretching above
        Eigen::Vector3d stretchTorqueNeighA = (0.5 * length) * springForceNeighA.cross(dirVector);
        Eigen::Vector3d stretchTorqueNeighB = (0.5 * length) * springForceNeighB.cross(-dirVector);
        // set velocity and angular momentum to zero for complete overdampining
        v = Eigen::Vector3d(0, 0, 0);
        L = Eigen::Vector3d(0, 0, 0);

        return {
                springForceNeighA + springForceNeighB,
                springTorqueNeighA + springTorqueNeighB + stretchTorqueNeighA + stretchTorqueNeighB
        };
    } else {
        return std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    }

}