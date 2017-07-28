/*
 * cCell.cpp
 *
 *  Created on: May 29, 2017
 *      Author: siegbahn
 */

#include "cCell.h"

cell::cell(
        sGlobalVars &iGlobals,
        functor_cell_membraneCreation *iMembraneF,
        functor_cell_filamentCreation *iFilamentF
) :
        cell_base(iGlobals),
        filamentF(iFilamentF),
        membraneF(iMembraneF),
        guiGroup(globals.guiMain->register_group("Cell")),
        x(guiGroup->register_setting<double>("X", false, 0, 500, 250)),
        y(guiGroup->register_setting<double>("Y", false, 0, 500, 250)) {
    membraneF->setup(*this);
    filamentF->setup(*this);
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

std::set<membrane_container *> &cell::get_membranes() {
    return membranes;
}

std::set<filament_base *> &cell::get_filaments() {
    return filaments;
}

std::set<volume_base *> &cell::get_volumes() {
    return volumes;
}

double &cell::get_x() {
    return x;
}

double &cell::get_y() {
    return y;
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
    // rebuild
    guiGroup->forceVariableUpdate();
    membraneF->setup(*this);
    filamentF->setup(*this);
}

void cell::make_timeStep(double &dT) {
    // filament creator makes time step
    filamentF->make_timeStep(dT, this);
    // filaments make time step, is more complicated since they get
    for (std::set<filament_base *>::iterator it = filaments.begin(); it != filaments.end();) {
        if ((*it)->make_timeStep(dT)) {
            delete *it;
            filament_base *del = *it;
            del = nullptr;
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

functor_cell_base::functor_cell_base(
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) :
        globals(iGlobals),
        guiGroup(globals.guiMain->register_group(iName)),
        guiFunctorGroup(globals.guiC->register_gui(iFunctorGroupName)) {

}

functor_cell_base::~functor_cell_base() {

}

void functor_cell_base::register_functor(physic::functor *iFunctor) {
    functors.insert(iFunctor);
}

mygui::gui *&functor_cell_base::get_guiFunctor() {
    return guiFunctorGroup;
}

/* Filament creation functors*/
functor_cell_filamentCreation::functor_cell_filamentCreation(
        sGlobalVars &iGlobals
) :
        functor_cell_base(iGlobals, "Filaments", "Forces"),
        randomReal(globals.rndC->register_random("uniform_real_distribution", 0.1, 1)),
        maxCount(guiGroup->register_setting<unsigned>("Count", true, 1, 1000, 100)),
        maxSpeed(guiGroup->register_setting<double>("Speed", true, 0, 0.05, 0.01)),
        maxLength(guiGroup->register_setting<double>("Length", true, 1, 200, 100)),
        maxLifeTime(guiGroup->register_setting<double>("Life Time", true, 0, 1000, 500)),
        maxStallingForce(guiGroup->register_setting<double>("Stalling Force", true, 0, 20, 10)) {
}

functor_cell_filamentCreation::~functor_cell_filamentCreation() {
    globals.rndC->unregister_random(randomReal);
}

void functor_cell_filamentCreation::setup(cell &iCell) {
    for (unsigned long long i = 0; i < maxCount; i++) {
        create_filament(iCell);
    }
}

void functor_cell_filamentCreation::make_timeStep(double &dT, cell *iCell) {
    long diff = maxCount - iCell->get_filaments().size();
    if (diff > 0) {
        for (unsigned i = 0; i < diff; i++) {
            create_filament(*iCell);
        }
    }
}

filament_base *functor_cell_filamentCreation::create_filament(cell &iCell) {
    pair<Eigen::Vector3d, membrane_part *> pos = find_creationPosition(iCell);
    actin *newActin = new actin(
            globals,
            iCell,
            pos.first,
            find_tmVelocity(iCell, *pos.second),
            find_maxLength(iCell),
            find_lifeTime(iCell),
            find_stallingForce(iCell),
            functors
    );
    iCell.register_filament(newActin);
    return newActin;
}

pair<Eigen::Vector3d, membrane_part *> functor_cell_filamentCreation::find_creationPosition(cell &iCell) {
    // get membrane and membrane parts
    auto &membranes = iCell.get_membranes();
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

Eigen::Vector3d functor_cell_filamentCreation::find_tmVelocity(cell &iCell, membrane_part &iMembrane) {
    double deg = PI * (randomReal->draw<double>() - 0.5);
    double speed = maxSpeed * randomReal->draw<double>();
    Eigen::AngleAxis<double> rot(deg, Eigen::Vector3d(0, 0, 1));
    return Eigen::Vector3d(speed * (rot * (-1 * iMembrane.get_normal())));
}

double functor_cell_filamentCreation::find_maxLength(cell &iCell) {
    return maxLength * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_lifeTime(cell &iCell) {
    return maxLifeTime * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_stallingForce(cell &iCell) {
    return maxStallingForce * randomReal->draw<double>();
}

functor_cell_membraneCreation::functor_cell_membraneCreation(sGlobalVars &iGlobals) :
        functor_cell_base(iGlobals, "Membrane", "Forces"),
        radius(guiGroup->register_setting<double>("Radius", false, 0, 200, 150)),
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 20, 200, 20)) {
}

functor_cell_membraneCreation::~functor_cell_membraneCreation() {

}

void functor_cell_membraneCreation::setup(cell &iCell) {
    guiGroup->forceVariableUpdate();
    // create new mebrane
    auto &membranes = iCell.get_membranes();
    membrane_container *newMembrane = new membrane_container(globals, iCell);
    membranes.insert(newMembrane);
    // get some data for membrane parts creation
    auto &parts = newMembrane->get_parts();
    auto x = iCell.get_x();
    auto y = iCell.get_y();
    x = std::min(max(x, radius), (double) globals.settings.sideLength);
    y = std::min(max(y, radius), (double) globals.settings.sideLength);
    // create membrane parts in circular shape
    double dAngle = 2 * PI / (double) resolution;
    for (unsigned long long i = 0; i < resolution; i++) {
        parts.push_back(new membrane_part(
                globals,
                iCell,
                radius * cos(i * dAngle) + x,
                radius * sin(i * dAngle) + y,
                radius * cos((i + 1) * dAngle) + x,
                radius * sin((i + 1) * dAngle) + y,
                functors
        ));
    };
    for (unsigned long long i = 0; i < resolution; i++) {
        std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &sharedPositions = parts[i]->get_sharedPositions();
        membrane_part_base *partA = parts[(i - 1) % resolution];
        membrane_part_base *partB = parts[(i + 1) % resolution];
        sharedPositions.first = &partA->get_positions()[1];
        sharedPositions.second = &partB->get_positions()[0];
        parts[i]->set_neighbours({partA, partB});
    };
}