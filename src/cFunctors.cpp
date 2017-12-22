/*
 * In this section are all force and torque functors defined one would like to hand to certain
 * cell elements. Important is that one does inherit from 'physic::functor' and that all functor
 * classes contain a virtual function 'calc' which returns a pair of 'Eigen::Vector3d' vectors
 * the first contains the 3d vector of the force and the second vector for the torque
 */
#include "cFunctors.h"

using namespace functor;

/*
 * filament filament friction
 */

/*

ffFriction::ffFriction(mygui::gui *&iGui) :
        functor(),
        guiGroup(iGui->register_group("F-F Friction")),
        activated(guiGroup->register_setting<bool>("Active", true, true)),
        frictionCoeff(guiGroup->register_setting<double>("Friction Coefficient", true, 0, 1, 1)),
        angleCoeff(guiGroup->register_setting<double>("Angle Coefficient", true, 0, 1, 1)) {

}

functor::ffFriction::~ffFriction() = default;

std::pair<Eigen::Vector3d, Eigen::Vector3d> ffFriction::calc(
        const Eigen::Vector3d &X,
        const Eigen::Vector3d &v,
        const double &R,
        const Eigen::Vector3d &L,
        physic::RigidBody3d &rigidBody
) {
    if (activated) {
        auto filament = dynamic_cast<filament_base *>(rigidBody.get_object());
        auto &intersectors = filament->get_intersectors();
        auto &positions = filament->get_positions();
        Eigen::Vector3d com = 0.5 * (positions[0] + positions[1]);
        Eigen::Vector3d force(0, 0, 0);
        Eigen::Vector3d torque(0, 0, 0);
        // run through all elements which intersect this filament
        for (auto &it : intersectors) {

            auto &el = it.first;
            auto &pos = *it.second;
            // check if element is actually a filament
            if (el) {
                if (auto *element = dynamic_cast<filament_base *>(el)) {
                    Eigen::Vector3d vNorm = element->get_rigidBody().get_v().normalized();
                    force += frictionCoeff * vNorm;
                    torque += frictionCoeff * (pos - com).cross(vNorm);
                }
            }
        }
        return {force, torque};
    }
    return {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
}

 */

/*
 * filament membrane collision
 */

/*

fmCollision::fmCollision(mygui::gui *&iGui) :
        functor(),
        guiGroup(iGui->register_group("F-M Collision")),
        activated(guiGroup->register_setting<bool>("Active", true, true)) {

}

fmCollision::~fmCollision() = default;

std::pair<Eigen::Vector3d, Eigen::Vector3d>
fmCollision::calc(
        const Eigen::Vector3d &X,
        const Eigen::Vector3d &v,
        const double &R,
        const Eigen::Vector3d &L,
        physic::RigidBody3d &rigidBody
) {
    if (activated) {
        auto _filament = dynamic_cast<actin *>(rigidBody.get_object());
        auto &_tmv = _filament->get_tmVelocity();
        for (auto &it : _filament->get_intersectors()) {
            if (auto _el = dynamic_cast<membrane_part_base *>(it.first)) {
                rigidBody.set_velocity(-(rigidBody.get_v() - _tmv));
            }
        }
        /*
        auto filament = dynamic_cast<filam>->get_object();
        // get intersecting elelments and position data
        auto &intersectors = filament->get_intersectors();
        auto &If = rigidBody->get_I();
        auto &Mf = rigidBody->get_M();
        auto &vf = rigidBody->get_v();
        auto &Rf = rigidBody->get_R();
        auto wf = R * If.cwiseInverse();
        auto &positions = filament->get_positions();
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
        // run through all intersecting elements and check if one of them is a membrane part
        for (auto &it : intersectors) {
            auto &el = it.first;
            auto &pos = it.second;
            if (auto *element = dynamic_cast<membrane_part_base *>(el)) {
                auto &Im = element->get_rigidBody().get_I();
                auto &Mm = element->get_rigidBody().get_M();
                auto &vm = element->get_rigidBody().get_v();
                auto &Rm = element->get_rigidBody().get_R();
                auto wm = R * Im.cwiseInverse();
                // calculated conserved quanitities
                auto Pt = Mf * vf + Mm * vm;
                double Et = 0.5 *
                            (Mf * vf.squaredNorm() + Mm * vm.squaredNorm() + wf.transpose() * Rf + wm.transpose() * Rm);
                double Rt = Rf + Rm;


            }
        }
        return {force, torque};

        return {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
    } else {
        return {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
    }
}


*/

/*
 * Constant force pushing on the fillaments ends
 */

fConstantForce::fConstantForce(mygui::gui *&iGui) :
        guiGroup(iGui->register_group("Constant Force")),
        activated(guiGroup->register_setting<bool>("Active", true, true)),
        factor(guiGroup->register_setting<double>("Factor", true, 0, 1, 0.01)),
        isAngle(guiGroup->register_setting<bool>("Angle Dependency", true, false))
    {
}

fConstantForce::~fConstantForce() = default;

std::pair<Eigen::Vector3d, Eigen::Vector3d>
fConstantForce::calc(
        const Eigen::Vector3d &X,
        const Eigen::Vector3d &v,
        const double &R,
        const Eigen::Vector3d &L,
        stokes::Solver &solver
) {
    auto _filament = dynamic_cast<filament_base *>(solver.get_object());
    auto &_X = _filament->get_positions();
    Eigen::Vector3d _dirVF = (_X[0] - _X[1]).normalized();
    auto _factorA = 1.0d;
    auto _factorB = 1.0d;
    /*if (isAngle) {
        auto &_prevMemPos = _filament->get_connectedMembraneLinker()->prevMembrane()->get_positions();
        auto &_nextMemPos = _filament->get_connectedMembraneLinker()->nextMembrane()->get_positions();
        Eigen::Vector3d _dirVP = _prevMemPos[0] - _prevMemPos[1];
        Eigen::Vector3d _dirVN = _nextMemPos[1] - _nextMemPos[0];
        auto _angleF = bmath::angleVector2d(Eigen::Vector3d(1,0,0), _dirVF);
        auto _angleP = bmath::angleVector2d(Eigen::Vector3d(1,0,0), _dirVP);
        auto _angleN = bmath::angleVector2d(Eigen::Vector3d(1,0,0), _dirVN);

        _factorA =  * cos(_angleA);
        _factorB = factor * sin(_angleB);
        if (_angleA > 0.5d * M_PI) {
            _factorA = 0;
        }
        if (_angleB > 0.5d * M_PI) {
            _factorB = 0;
        }
    }*/

    Eigen::Vector3d _F(_dirVF * 0.5 * factor * (_factorA + _factorB));
    return {_F, Eigen::Vector3d(0, 0, 0)};
}
