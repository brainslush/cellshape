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
        double iX2, double iY2
) : membrane_part_base(iGlobals, iCell, iX1, iY1, iX2, iY2) {

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

};


/***************************
 * membrane base
 ***************************/

// circular membrane
membrane_base::membrane_base(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        double iX,
        double iY,
        double iRadius,
        unsigned long long iResolution
) : cellcomponents_base(iGlobals, iCell) {
    double dAngle = 2 * PI / (double) iResolution;
    for (unsigned long long i = 0; i < iResolution; i++) {
        parts.push_back(new membrane_part(
                iGlobals,
                iCell,
                iRadius * cos(i * dAngle) + iX,
                iRadius * sin(i * dAngle) + iY,
                iRadius * cos((i + 1) * dAngle) + iX,
                iRadius * sin((i + 1) * dAngle) + iY
        ));
    };
    for (unsigned long long i = 0; i < iResolution; i++) {
        std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &sharedPositions = parts[i]->get_sharedPositions();
        membrane_part_base* partA = parts[(i - 1) % iResolution];
        membrane_part_base* partB = parts[(i + 1) % iResolution];
        sharedPositions.first = &partA->get_positions()[1];
        sharedPositions.second = &partB->get_positions()[0];
        parts[i]->set_neighbours({partA, partB});
    };
    update_area();
    update_length();
};

membrane_base::~membrane_base() {
    for (auto &it : parts) {
        delete it;
        it = nullptr;
    }
}

/* calculate area */
void membrane_base::update_area() {
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
void membrane_base::update_length() {
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
void membrane_base::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    for (unsigned long long i = 0; i < parts.size(); i++) {
        parts[i]->obtain_visualObjs(oVisualComponents);
    }
}

/* get volume */
double &membrane_base::get_area() {
    update_area();
    return area;
}

double &membrane_base::get_length() {
    update_length();
    return length;
}

std::vector<membrane_part *> &membrane_base::get_parts() {
    return parts;
}

/* variableUpdate feature of the membrane */
void membrane_base::make_timeStep(double &dT) {

}
