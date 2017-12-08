/*
 * cMembrane.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cMembrane.h"


/***************************
 * membrane part
 ***************************/

membrane_part::membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        double iX1, double iY1,
        double iX2, double iY2,
        std::set<stokes::functor *> &iFunctors
) :
        membrane_part_base(iGlobals, iCell) {
    positions.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.emplace_back(Eigen::Vector3d(iX2, iY2, 0));
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    update_normal();
};

membrane_part::membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        const Eigen::Vector3d &iPosS,
        const Eigen::Vector3d &iPosE,
        std::set<stokes::functor *> &iFunctors
) :
        membrane_part_base(iGlobals, iCell) {
    positions.clear();
    positions.emplace_back(iPosS);
    positions.emplace_back(iPosE);
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    update_normal();
};

membrane_part::~membrane_part() {
    //globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
};

void membrane_part::set_sharedPositions(const std::pair<Vector3d *, Vector3d *> &iSharedPos) {
    sharedPositions = iSharedPos;
}

std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &membrane_part::get_sharedPositions() {
    return sharedPositions;
}

Eigen::Vector3d &membrane_part::get_normal() {
    return normal;
}

void membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

void membrane_part::make_timeStep(const double &dT) {
    if (solver) {
        //rigidBody->do_timeStep(dT);
    } else {
        std::cout << "membrane_part w/o RigidBody\n";
    }
}

double membrane_part::get_length() {
    return (get_positions()[0] - get_positions()[1]).norm();
}

void membrane_part::update_normal() {
    normal = (Eigen::Vector3d(0, 0, -1).cross(positions[1] - positions[0])).normalized();
}

/***************************
 * arc membrane
 ***************************/

arc_membrane_part::arc_membrane_part(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        double iX1,
        double iY1,
        double R,
        double angleB,
        double angleE,
        std::set<stokes::functor *> &iFunctors
) :
        membrane_part_base(iGlobals, iCell) {
    positions.clear();
    parameters.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    parameters.emplace_back(R);
    parameters.emplace_back(angleB);
    parameters.emplace_back(angleE);
    associatedVisualObj = new visual_arcCircle(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    //globals.grid->register_component(this);
}

arc_membrane_part::~arc_membrane_part() {

}

Eigen::Vector3d arc_membrane_part::get_normal(const double &iDeg) {
    return Eigen::Vector3d(cos(iDeg), sin(iDeg), 0);
}

void arc_membrane_part::make_timeStep(const double &dT) {

}

double arc_membrane_part::get_length() {
    auto _diff = std::abs(get_parameters()[2] - get_parameters()[1]);
    return get_parameters()[0] * _diff;
}

void arc_membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}
