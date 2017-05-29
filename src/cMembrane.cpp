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
    sGlobalVars& iGlobals,
    cell_base& iCell,
    double iX1,double iY1,
    double iX2,double iY2
): cellcomponents_base(iGlobals,iCell) {
    positions.clear();
    positions.push_back(Eigen::Vector3d(iX1,iY1,0));
    positions.push_back(Eigen::Vector3d(iX2,iY2,0));
    neighbours.push_back(this);
    neighbours.push_back(this);
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0.0,0.0,0.0);
    associatedVisualObj->set_fillColor(0.0,0.0,0.0);
    this->add_ignoreIntersect(typeid(*this).hash_code());
    iGlobals.grid->register_component(this);
};
membrane_part::~membrane_part(){

};
void membrane_part::obtain_visualObjs(std::vector<visual_base*>& iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}
void membrane_part::set_neighbours(membrane_part& iPartA,membrane_part& iPartB){
    neighbours[0] = &iPartA;
    neighbours[1] = &iPartB;
};
void membrane_part::make_timeStep(double& dT){

};


/***************************
 * membrane base
 ***************************/

// circular membrane
membrane_base::membrane_base(
    sGlobalVars& iGlobals,
    cell_base& iCell,
    double iX,
    double iY,
    double iRadius,
    unsigned long long iResolution
): cellcomponents_base(iGlobals,iCell) {
    double dAngle = 2*PI/(double)iResolution;
    for (unsigned long long i = 0; i < iResolution; i++) {
        parts.push_back(new membrane_part(
            iGlobals,
            iCell,
            iRadius * cos(i    *dAngle) + iX,
            iRadius * sin(i    *dAngle) + iY,
            iRadius * cos((i+1)*dAngle) + iX,
            iRadius * sin((i+1)*dAngle) + iY
        ));
    };
    for (unsigned long long i = 0; i < iResolution; i++) {
        parts[i]->set_neighbours(*parts[(i-1)%iResolution],*parts[(i+1)%iResolution]);
    };
    //area.set_updated(false);
    //length.set_updated(false);
    update_area();
    update_length();
};
membrane_base::~membrane_base() {
    for (unsigned long long i = 0; i < parts.size(); i++) {
        delete parts[i];
    }
}

/* calculate area */
void membrane_base::update_area() {
  //if(!area.isUpdated()) {
      double temp = 0;
      /* calculate 2D volume aka the area */
      for(auto& it : parts) {
          Eigen::Vector3d& posA = it->get_positions()[0];
          Eigen::Vector3d& posB = it->get_positions()[1];
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
        for (auto& it : parts) {
            Eigen::Vector3d& posA = it->get_positions()[0];
            Eigen::Vector3d& posB = it->get_positions()[1];
            temp += (posA - posB).norm();
        }
        length = temp;
        //length.set_updated(true);
    //}
}

/* */
void membrane_base::obtain_visualObjs(std::vector<visual_base*>& oVisualComponents) {
    for(unsigned long long i = 0; i < parts.size(); i++) {
        parts[i]->obtain_visualObjs(oVisualComponents);
    }
}

/* get volume */
double& membrane_base::get_area() {
    update_area();
    return area;
}
double& membrane_base::get_length() {
    update_length();
    return length;
}
/* update feature of the membrane */
void membrane_base::make_timeStep(double& dT) {

}
