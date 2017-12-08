

#include "cCell.h"

/*
 * cell class which holds all the cell elements
 */

cell::cell(
        sGlobalVars &iGlobals,
        functor_cell_membraneCreation *iMembraneF,
        functor_cell_filamentCreation *iFilamentF,
        functor_cell_linkerCreation *iLinkerF
) :
        cell_base(iGlobals),
        filamentF(iFilamentF),
        membraneF(iMembraneF),
        linkerF(iLinkerF),
        guiGroup(globals.guiMain->register_group("Cell")),
        x(guiGroup->register_setting<double>("X", false, 0, 500, 250)),
        y(guiGroup->register_setting<double>("Y", false, 0, 500, 250)) {
    membraneF->setup(*this);
    filamentF->setup(*this);
}

cell::~cell() {
    delete membrane;
    membrane = nullptr;
    for (auto _it : filaments) {
        delete _it;
        _it = nullptr;
    }
    for (auto _it : volumes) {
        delete _it;
        _it = nullptr;
    }
    for (auto _it : linkers) {
        delete _it;
        _it = nullptr;
    }
}

/*
 * cell class getters
 */

std::set<filament_base *> &cell::get_filaments() {
    return filaments;
}

std::set<volume_base *> &cell::get_volumes() {
    return volumes;
}

functor_filament_base *cell::get_filamentFunctor() {
    return filamentF;
}

functor_membrane_base *cell::get_membraneFunctor() {
    return membraneF;
}

functor_linker_base *cell::get_linkerFunctor() {
    return linkerF;
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
    membrane->obtain_visualObjs(oVisualComponents);
    for (auto &_it : filaments) {
        _it->obtain_visualObjs(oVisualComponents);
    }
    for (auto &_it : volumes) {
        _it->obtain_visualObjs(oVisualComponents);
    }
    for (auto &_it : linkers) {
        _it->obtain_visualObjs(oVisualComponents);
    }
}

void cell::register_filament(filament_base *iFilament) {
    filaments.insert(iFilament);
}

void cell::unregister_filament(filament_base *iFilament) {
    filaments.erase(iFilament);
}

void cell::reset() {
    // delete filaments
    for (auto _it: filaments) {
            delete _it;
            _it = nullptr;
    }
    filaments.clear();
    // delete membrane
    delete membrane;
    membrane = nullptr;
    // delete volumes
    for (auto _it: volumes) {
        if (_it) {
            delete _it;
            _it = nullptr;
        }
    }
    volumes.clear();
    // delete linkers
    for (auto _it: linkers) {
        if (_it) {
            delete _it;
            _it = nullptr;
        }
    }
    linkers.clear();
    // rebuild
    guiGroup->forceVariableUpdate();
    membraneF->setup(*this);
    filamentF->setup(*this);
}

/*
 * simulate single time step
 */

void cell::make_timeStep(double &dT) {
    // filament creator makes time step
    filamentF->make_timeStep(dT, this);
    // let linkers make a time step
    for (auto &_it: linkers) {
        _it->make_timeStep(dT);
    }

    membraneF->make_timeStep(dT, this);
    // volumes make a time step
    for (auto &_it: volumes) {
        _it->make_timeStep(dT);
    }

}

void cell::register_linker(linker_base *iLinker) {
    linkers.insert(iLinker);
}

void cell::unregister_linker(linker_base *iLinker) {
    linkers.erase(iLinker);
}