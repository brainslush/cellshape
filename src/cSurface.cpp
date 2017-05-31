/*
 * cSurface.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cSurface.h"

boost::random::mt19937 RandomGen;

/***************************
 * surface_borders
 ***************************/

surface_border::surface_border(
    sGlobalVars& iGlobals,
    simple_surface* iSurface,
    Eigen::Vector3d iStart,
    Eigen::Vector3d iEnd
): matrix_base(iGlobals) {
    surface = iSurface;
    positions.clear();
    positions.push_back(iStart);
    positions.push_back(iEnd);
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_fillColor(1.0,1.0,1.0);
    associatedVisualObj->set_color(1.0,1.0,1.0);
    iGlobals.grid->register_component(this);
}
surface_border::~surface_border() {
    delete associatedVisualObj;
    associatedVisualObj = NULL;
}
void surface_border::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
    oVisualComponents.push_back(associatedVisualObj);
}

/***************************
 * simple surface
 ***************************/

simple_surface::simple_surface(
    sGlobalVars& iGlobals,
    double iSideLength
): matrix_base(iGlobals) {
    sideLength = iSideLength;
    positions.clear();
    positions.push_back(Eigen::Vector3d(0,0,0));
    positions.push_back(Eigen::Vector3d(0,iSideLength,0));
    positions.push_back(Eigen::Vector3d(iSideLength,iSideLength,0));
    positions.push_back(Eigen::Vector3d(iSideLength,0,0));
    borders.push_back(new surface_border(iGlobals,this,positions[0],positions[1]));
    borders.push_back(new surface_border(iGlobals,this,positions[1],positions[2]));
    borders.push_back(new surface_border(iGlobals,this,positions[2],positions[3]));
    borders.push_back(new surface_border(iGlobals,this,positions[3],positions[0]));
}
simple_surface::~simple_surface(){
    for (auto& it : borders) {
        delete it;
        it = NULL;
    }
    for (auto& it : facs) {
        delete it;
        it = NULL;
    }
}
void simple_surface::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
    for (auto& it : borders) {
        it->obtain_visualObjs(oVisualComponents);
    }
    for (auto& it : facs) {
        it->obtain_visualObjs(oVisualComponents);
    }
}
void simple_surface::create_facs(unsigned iType, unsigned long long iCount, double iRadius) {
    boost::random::uniform_real_distribution <double> rndPosX(positions[0](0) + iRadius, positions[2](0) - iRadius);
    boost::random::uniform_real_distribution <double> rndPosY(positions[0](1) + iRadius, positions[2](1) - iRadius);
    switch(iType) {
        case 0: {
            for (unsigned long long i = 0; i < iCount; i++) {
                facs.push_back(new fac(globals,iRadius,rndPosX(RandomGen),rndPosY(RandomGen)));
            }
        }
        break;
    }
}
