/*
 * cCell.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cCell.h"

cell::cell(
    sGlobalVars& iGlobals,
    double iX,
    double iY,
    unsigned long long iResolution
): cell_base(iGlobals) {
    membranes.insert(new membrane_base(iGlobals,*this,iX,iY,200,iResolution));
    maxFillamentLength = 1;
}
cell::~cell(){
    for (auto& it : membranes) {
        delete it;
    }
    for (auto& it : fillaments) {
        delete it;
    }
    for (auto& it : volumes) {
        delete it;
    }
}
void cell::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
    for(auto& it : membranes) {
        it->obtain_visualObjs(oVisualComponents);
    }
    for(auto& it : fillaments) {
        it->obtain_visualObjs(oVisualComponents);
    }
    for(auto& it : volumes) {
        it->obtain_visualObjs(oVisualComponents);
    }
}
void cell::create_fillament() {
    /* Needs to be enhanced */
    fillaments.insert(new actin(globals,*this,Eigen::Vector3d(90,90,0),Eigen::Vector3d(0.1,0.1,0),100,2000,10));
}
void cell::destory_fillament(fillament_base * iFillament) {
    fillaments.erase(iFillament);
    delete iFillament;
}
void cell::make_timeStep(double& dT) {
    for(auto it: fillaments) {
      it->make_timeStep(dT);
    }
    for(auto it: membranes) {
      it->make_timeStep(dT);
    }
    for(auto it: volumes) {
      it->make_timeStep(dT);
    }
}
