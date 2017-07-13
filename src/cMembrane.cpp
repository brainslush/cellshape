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
        std::set<physic::functor *> &iFunctors
) : membrane_part_base(iGlobals, iCell, iX1, iY1, iX2, iY2) {
    double mass = 0.1;
    double length = (positions[0] - positions[1]).norm();
    double I = 0.83333333 * mass * length * length;
    Eigen::Matrix3d mI;
    mI << I, 0, 0,
            0, I, 0,
            0, 0, 0;
    Eigen::Matrix3d mI2 = mI.inverse();
    Eigen::Vector3d mI3 = mI2.diagonal();
    rigidBody = physic::RigidBody3d(
            Eigen::Vector3d(0, 0, 0),
            Eigen::Quaterniond(0, 0, 0, 1),
            mI,
            0.1,
            0.1,
            &iFunctors
    );
    normal = Eigen::Vector3d(0, 0, -1).cross(positions[1] - positions[0]);
};

membrane_part::~membrane_part() {
    globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
};

Eigen::Vector3d &membrane_part::get_normal() {
    return normal;
}

void membrane_part::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

void membrane_part::make_timeStep(double &dT) {
    rigidBody.do_timeStep(dT, this);
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
        Eigen::Vector3d &posA = it->get_positions()[0];
        Eigen::Vector3d &posB = it->get_positions()[1];
        temp += -1 * posB(0) * posA(1) + posA(0) * posB(1);
    };
    area = temp;
    //area.set_updated(true);
    //}
}

/* calculate length */
void membrane_container::update_length() {
    //if (!length.isUpdated()) {
    double temp = 0;
    for (auto &it : parts) {
        Eigen::Vector3d &posA = it->get_positions()[0];
        Eigen::Vector3d &posB = it->get_positions()[1];
        temp += (posA - posB).norm();
    }
    length = temp;
    //length.set_updated(true);
    //}
}

/* */
void membrane_container::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    for (unsigned long long i = 0; i < parts.size(); i++) {
        parts[i]->obtain_visualObjs(oVisualComponents);
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

std::vector<membrane_part *> &membrane_container::get_parts() {
    return parts;
}

/* variableUpdate feature of the membrane */
void membrane_container::make_timeStep(double &dT) {

}
