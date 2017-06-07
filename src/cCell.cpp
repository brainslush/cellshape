/*
 * cCell.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cCell.h"

cell::cell(
        sGlobalVars &iGlobals,
        double iX,
        double iY,
        unsigned long long iResolution,
        functor_cell_filamentCreation *iFunctor
) :
        cell_base(iGlobals),
        filamentF(iFunctor) {
    membranes.insert(new membrane_base(iGlobals, *this, iX, iY, 200, iResolution));
    filamentF->setup(this);
}

cell::~cell() {
    for (auto &it : membranes) {
        delete it;
    }
    for (auto &it : filaments) {
        delete it;
    }
    for (auto &it : volumes) {
        delete it;
    }
}

functor_cell_filamentCreation *&cell::get_filamentCreationFunctor() {
    return filamentF;
}

std::set<membrane_base *> &cell::get_membranes() {
    return membranes;
}

std::set<filament_base *> &cell::get_filaments() {
    return filaments;
}

std::set<volume_base *> &cell::get_volumes() {
    return volumes;
}

void cell::set_filamentCreationFunctor(functor_cell_filamentCreation *iFunctor) {
    filamentF = iFunctor;
}

void cell::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    for (auto &it : membranes) {
        it->obtain_visualObjs(oVisualComponents);
    }
    for (auto &it : filaments) {
        it->obtain_visualObjs(oVisualComponents);
    }
    for (auto &it : volumes) {
        it->obtain_visualObjs(oVisualComponents);
    }
}

void cell::add_filament(filament_base *iFilament) {
    filaments.insert(iFilament);
}

void cell::create_filament() {
    /* Needs to be enhanced */
    //filaments.insert(new actin(globals,*this,Eigen::Vector3d(90,90,0),Eigen::Vector3d(0.1,0.1,0),100,2000,10));
    filamentF->setup(this);
}

void cell::destory_filament(filament_base *iFilament) {
    filaments.erase(iFilament);
    delete iFilament;
}

void cell::make_timeStep(double &dT) {
    for (auto it: filaments) {
        it->make_timeStep(dT);
    }
    for (auto it: membranes) {
        it->make_timeStep(dT);
    }
    for (auto it: volumes) {
        it->make_timeStep(dT);
    }
}

/* Filament creation functors*/
functor_cell_filamentCreation::functor_cell_filamentCreation(
        sGlobalVars &iGlobals,
        unsigned long long iMaxCount,
        double iMaxSpeed,
        double iMaxLength,
        double iMaxLifeTime,
        double iMaxStallingForce
) :
        globals(iGlobals),
        maxCount(iMaxCount),
        maxSpeed(iMaxSpeed),
        maxLength(iMaxLength),
        maxLifeTime(iMaxLifeTime),
        maxStallingForce(iMaxStallingForce) {
    randomReal = globals.rndC->register_random("uniform_01");
}

functor_cell_filamentCreation::~functor_cell_filamentCreation() {
    globals.rndC->unregister_random(randomReal);
}

void functor_cell_filamentCreation::setup(cell *iCell) {
    for (unsigned long long i = 0; i < maxCount; i++) {
        pair<Eigen::Vector3d, membrane_part *> pos = find_creationPosition(iCell);
        actin *newActin = new actin(
                globals,
                *iCell,
                pos.first,
                find_tmVelocity(iCell, pos.second),
                find_maxLength(iCell),
                find_lifeTime(iCell),
                find_stallingForce(iCell)
        );
        iCell->add_filament(newActin);
    }
}

void functor_cell_filamentCreation::make_timeStep(double &dT, cell *iCell) {

}

filament_base *functor_cell_filamentCreation::create_filament(cell *iCell) {
    return NULL;
}

pair<Eigen::Vector3d, membrane_part *> functor_cell_filamentCreation::find_creationPosition(cell *iCell) {
    auto &membranes = iCell->get_membranes();
    //auto& filaments = iCell->get_filaments();
    //auto& volumes = iCell->get_volumes();
    auto &parts = (*membranes.begin())->get_parts();

    double length = (*membranes.begin())->get_length() * randomReal->draw<double>();
    auto it = parts.begin();
    double currLength = (*it)->get_length();
    while (currLength < length) {
        it++;
        currLength += (*it)->get_length();
    }
    length -= currLength;
    auto &pos = (*it)->get_positions();
    Eigen::Vector3d returnVec = pos[0] + length * (pos[1] - pos[0]).normalized();

    return std::make_pair(returnVec, *it);
}

Eigen::Vector3d functor_cell_filamentCreation::find_tmVelocity(cell *iCell, membrane_part *iMembrane) {
    double deg = PI * (randomReal->draw<double>() - 0.5);
    double speed = maxSpeed * randomReal->draw<double>();
    Eigen::AngleAxis<double> rot(deg, Eigen::Vector3d(0, 0, 1));
    return Eigen::Vector3d(speed * (rot * (-1 * iMembrane->get_normal())));
}

double functor_cell_filamentCreation::find_maxLength(cell *iCell) {
    return maxLength * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_lifeTime(cell *iCell) {
    return maxLifeTime * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_stallingForce(cell *iCell) {
    return maxStallingForce * randomReal->draw<double>();
}
