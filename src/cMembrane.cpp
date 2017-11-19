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
) : membrane_part_base(iGlobals, iCell, iX1, iY1, iX2, iY2) {
    normal = Eigen::Vector3d(0, 0, -1).cross(positions[1] - positions[0]);
};

membrane_part::~membrane_part() {
    //globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
};

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
};


/***************************
 * membrane base
 ***************************/

// circular membrane
membrane_container::membrane_container(
        sGlobalVars &iGlobals,
        cell_base &iCell
) : cellcomponents_base(iGlobals, iCell) {
    update_area();
    update_length();
};

membrane_container::~membrane_container() {
    for (auto &it : parts) {
        delete it;
        it = nullptr;
    }
}

/* calculate area */
void membrane_container::update_area() {
    //if(!area.isUpdated()) {
    double temp = 0;
    /* calculate 2D volume aka the area */
    for (auto &it : parts) {
        auto &posA = it->get_positions()[0];
        auto &posB = it->get_positions()[1];
        temp += -1 * posB(0) * posA(1) + posA(0) * posB(1);
    };
    area = temp;
}

/* calculate length */
void membrane_container::update_length() {
    //if (!length.isUpdated()) {
    length = 0;
    for (auto &_it : parts) {
        auto _l = _it->get_length();
        length += _l;
    }
}

/* */
void membrane_container::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    for (auto &it : parts) {
        it->obtain_visualObjs(oVisualComponents);
    }
}

/* get volume */
double &membrane_container::get_area() {
    update_area();
    return area;
}

double &membrane_container::get_length() {
    update_length();
    return length;
}

std::vector<membrane_part_base *> &membrane_container::get_parts() {
    return parts;
}

/* variableUpdate feature of the membrane */
void membrane_container::make_timeStep(const double &dT) {

}

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

Eigen::Vector3d arc_membrane_part::get_normal(const double &deg) {
    return Eigen::Vector3d(cos(deg),sin(deg),0);
}

void arc_membrane_part::make_timeStep(const double &dT) {

}

double arc_membrane_part::get_length() {
    auto _diff = std::abs(get_parameters()[2] - get_parameters()[1]);
    return get_parameters()[0] * (_diff);
}

void arc_membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}
