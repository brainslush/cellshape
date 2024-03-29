/*
 * cSurface.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cSurface.h"

/***************************
 * surface_borders
 ***************************/

surface_border::surface_border(
        sGlobalVars &iGlobals,
        simple_surface *iSurface,
        Eigen::Vector3d iStart,
        Eigen::Vector3d iEnd
) : surface_border_base(iGlobals),
    surface(iSurface) {
    associatedVisualObj = new visual_line(this);
    positions.clear();
    positions.push_back(iStart);
    positions.push_back(iEnd);
    associatedVisualObj->set_fillColor(1.0, 1.0, 1.0);
    associatedVisualObj->set_color(1.0, 1.0, 1.0);
    globals.grid->register_component(this);
}

surface_border::~surface_border() {
    globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void surface_border::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    oVisualComponents.push_back(associatedVisualObj);
}

/***************************
 * simple surface
 ***************************/

simple_surface::simple_surface(
        sGlobalVars &iGlobals,
        double iSideLength
) : surface_base(iGlobals),
    randomReal(globals.rndC->register_random("uniform_01")),
    //guiGroup(globals.guiMain->register_group("")),
    //facCount(guiGroup->register_setting<unsigned>("FAC Count", false, 0, 100, 20)),
    //facRadius(guiGroup->register_setting<double>("FAC Radius", false, 0, 20, 10)),
    sideLength(iSideLength) {
    create_borders();
    //create_facs();
}

simple_surface::~simple_surface() {
    for (auto _it : borders) {
        delete _it;
        _it = nullptr;
    }
    for (auto _it : facs) {
        delete _it;
        _it = nullptr;
    }
}

void simple_surface::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    for (auto &_it : borders) {
        _it->obtain_visualObjs(oVisualComponents);
    }
    for (auto &_it : facs) {
        _it->obtain_visualObjs(oVisualComponents);
    }
}

void simple_surface::reset() {
    for (auto _it: borders) {
        delete _it;
        _it = nullptr;
    }
    borders.clear();
    for (auto _it: facs) {
        delete _it;
        _it = nullptr;
    }
    facs.clear();
    //guiGroup->forceVariableUpdate();
    create_borders();
    //create_facs();
}

void simple_surface::create_facs() {
    /*double _redSideLength = sideLength - 2 * facRadius;
    for (unsigned long long _i = 0; _i < facCount; _i++) {
        double _rndPosX = positions[0](0) + facRadius + randomReal->draw<double>() * _redSideLength;
        double _rndPosY = positions[0](1) + facRadius + randomReal->draw<double>() * _redSideLength;
        facs.push_back(new fac(globals, facRadius, _rndPosX, _rndPosY));
    }
     */
}

void simple_surface::create_borders() {
    positions.clear();
    positions.emplace_back(Eigen::Vector3d(0, 0, 0));
    positions.emplace_back(Eigen::Vector3d(0, sideLength, 0));
    positions.emplace_back(Eigen::Vector3d(sideLength, sideLength, 0));
    positions.emplace_back(Eigen::Vector3d(sideLength, 0, 0));
    borders.push_back(new surface_border(globals, this, positions[0], positions[1]));
    borders.push_back(new surface_border(globals, this, positions[1], positions[2]));
    borders.push_back(new surface_border(globals, this, positions[2], positions[3]));
    borders.push_back(new surface_border(globals, this, positions[3], positions[0]));
}