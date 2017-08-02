/*
 * cFac.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cFac.h"

/***************************
 * FAC base
 ***************************/

fac::fac(
        sGlobalVars &iGlobals,
        double iRadius,
        double iX,
        double iY
) : fac_base(iGlobals) {
    parameters.clear();
    parameters.push_back(iRadius);
    parameters.push_back(iRadius);
    positions.clear();
    positions.push_back(Eigen::Vector3d(iX, iY, 0));
    associatedVisualObj = new visual_ellipse(this);
    associatedVisualObj->set_color(1.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(1.0, 0.0, 0.0);
    globals.grid->register_component(this);
}

fac::~fac() {
    globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void fac::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    oVisualComponents.push_back(associatedVisualObj);
}

void fac::set_radius(double iRadius) {
    parameters.clear();
    parameters.push_back(iRadius);
    parameters.push_back(iRadius);
}

void fac::set_position(double iX, double iY) {
    positions.clear();
    positions.push_back(Eigen::Vector3d(iX, iY, 0));
}


