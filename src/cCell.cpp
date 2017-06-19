/*
 * cCell.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cCell.h"

cell::cell(
        sGlobalVars &iGlobals,
        functor_cell_filamentCreation *iFunctor
) :
        cell_base(iGlobals),
        filamentF(iFunctor),
        guiGroup(globals.guiBase->register_group("Cell")),
        x(guiGroup->register_setting<double>("X", false, 0, 500, 250)),
        y(guiGroup->register_setting<double>("Y", false, 0, 500, 250)),
        radius(guiGroup->register_setting<double>("Radius", false, 0, 200, 150)),
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 20, 200, 20)) {
    membranes.insert(new membrane_base(globals, *this, x, y, radius, resolution));
    filamentF->setup(this);
}

cell::~cell() {
    for (auto it : membranes) {
        delete it;
        it = nullptr;
    }
    for (auto it : filaments) {
        delete it;
        it = nullptr;
    }
    for (auto it : volumes) {
        delete it;
        it = nullptr;
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

void cell::register_filament(filament_base *iFilament) {
    filaments.insert(iFilament);
}

void cell::unregister_filament(filament_base *iFilament) {
    filaments.erase(iFilament);
}

void cell::unregister_filament(std::set<filament_base *>::iterator iIt) {
    filaments.erase(iIt);
}

void cell::reset() {
    for (auto it: filaments) {
        delete it;
        it = nullptr;
    }
    filaments.clear();
    for (auto it: membranes) {
        delete it;
        it = nullptr;
    }
    membranes.clear();
    for (auto it: volumes) {
        delete it;
        it = nullptr;
    }
    volumes.clear();
    guiGroup->forceVariableUpdate();
    x = std::min(max(x, radius), (double) globals.settings.sideLength);
    y = std::min(max(y, radius), (double) globals.settings.sideLength);
    membranes.insert(new membrane_base(globals, *this, x, y, radius, resolution));
    filamentF->setup(this);
}

void cell::make_timeStep(double &dT) {
    // filament creator makes time step
    filamentF->make_timeStep(dT, this);
    // filaments make time step
    for (std::set<filament_base *>::iterator it = filaments.begin(); it != filaments.end();) {
        if ((*it)->make_timeStep(dT)) {
            delete *it;
            filament_base *del = *it;
            del = NULL;
            it = filaments.erase(it);
        } else {
            it++;
        }
    }

    for (auto &it: membranes) {
        it->make_timeStep(dT);
    }
    for (auto &it: volumes) {
        it->make_timeStep(dT);
    }
}

/* Filament creation functors*/
functor_cell_filamentCreation::functor_cell_filamentCreation(
        sGlobalVars &iGlobals
) :
        globals(iGlobals),
        randomReal(globals.rndC->register_random("uniform_01")),
        guiGroup(globals.guiBase->register_group("Filaments")),
        maxCount(guiGroup->register_setting<unsigned>("Count", true, 1, 1000, 100)),
        maxSpeed(guiGroup->register_setting<double>("Speed", true, 0, 0.05, 0.01)),
        maxLength(guiGroup->register_setting<double>("Length", true, 1, 200, 100)),
        maxLifeTime(guiGroup->register_setting<double>("Life Time", true, 0, 1000, 500)),
        maxStallingForce(guiGroup->register_setting<double>("Stalling Force", true, 0, 20, 10)),
        guiForceGroup(guiGroup->register_group("Forces")),
        guiTorqueGroup(guiGroup->register_group("Torques")) {
}

functor_cell_filamentCreation::~functor_cell_filamentCreation() {
    globals.rndC->unregister_random(randomReal);
}

mygui::group *&functor_cell_filamentCreation::get_guiForceGroup() {
    return guiForceGroup;
}

mygui::group *&functor_cell_filamentCreation::get_guiTorqueGroup() {
    return guiTorqueGroup;
}

void functor_cell_filamentCreation::setup(cell *iCell) {
    for (unsigned long long i = 0; i < maxCount; i++) {
        create_filament(iCell);
    }
}

void functor_cell_filamentCreation::register_force(physic::functor *iFunctor) {
    forceFunctors.insert(iFunctor);
}

void functor_cell_filamentCreation::register_torque(physic::functor *iFunctor) {
    forceFunctors.insert(iFunctor);
}

void functor_cell_filamentCreation::make_timeStep(double &dT, cell *iCell) {
    long diff = maxCount - iCell->get_filaments().size();
    if (diff > 0) {
        for (unsigned i = 0; i < diff; i++) {
            create_filament(iCell);
        }
    }
}

filament_base *functor_cell_filamentCreation::create_filament(cell *iCell) {
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
    iCell->register_filament(newActin);
    return newActin;
}

pair<Eigen::Vector3d, membrane_part *> functor_cell_filamentCreation::find_creationPosition(cell *iCell) {
    // get membrane and membrane parts
    auto &membranes = iCell->get_membranes();
    auto &parts = (*membranes.begin())->get_parts();
    // determine new position along membrane
    double length = (*membranes.begin())->get_length() * randomReal->draw<double>();
    auto it = parts.begin();
    double currLength = (*it)->get_length();
    while (currLength < length) {
        it++;
        currLength += (*it)->get_length();
    }
    length -= currLength;
    auto &pos = (*it)->get_positions();

    return std::make_pair(Eigen::Vector3d(pos[0] + length * (pos[0] - pos[1]).normalized()), *it);
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
