

#include "cCell.h"

/*
 * cell class which holds all the cell elements
 */

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

/*
 * cell class getters
 */

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
    for (auto &_it : membranes) {
        _it->obtain_visualObjs(oVisualComponents);
    }
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

void cell::unregister_filament(std::set<filament_base *>::iterator iIt) {
    filaments.erase(iIt);
}

void cell::reset() {
    for (auto it: filaments) {
        if (it) {
            delete it;
            it = nullptr;
        }
    }
    filaments.clear();
    for (auto it: membranes) {
        if (it) {
            delete it;
            it = nullptr;
        }
    }
    membranes.clear();
    for (auto it: volumes) {
        if (it) {
            delete it;
            it = nullptr;
        }
    }
    volumes.clear();
    for (auto it: linkers) {
        if (it) {
            delete it;
            it = nullptr;
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
    // filaments make time step, is more complicated since they get
    std::vector<filament_base *> _delf;
    for (auto &_it: filaments) {
        if (_it->make_timeStep(dT)) {
            _delf.push_back(_it);
        }
    }
    // membrane make a time step
    for (auto &_it: membranes) {
        _it->make_timeStep(dT);
    }
    // volumes make a time step
    for (auto &_it: volumes) {
        _it->make_timeStep(dT);
    }
    for (auto &_it: linkers) {
        _it->make_timeStep(dT);
    }
    for (auto &_it : _delf) {
        filaments.erase(_it);
        delete _it;
        _it = nullptr;
    }
}

void cell::register_linker(linker_base *iLinker) {
    linkers.insert(iLinker);
}

void cell::unregister_filament(linker_base *iLinker) {
    linkers.erase(iLinker);
}

/*
 * base class for cell functors
 */

functor_cell_base::functor_cell_base(
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) :
        globals(iGlobals),
        guiGroup(globals.guiMain->register_group(std::move(iName))),
        guiFunctorGroup(globals.guiC->register_gui(std::move(iFunctorGroupName))) {

}

functor_cell_base::~functor_cell_base() = default;

void functor_cell_base::register_functor(stokes::functor *iFunctor) {
    functors.insert(iFunctor);
}

mygui::gui *&functor_cell_base::get_guiFunctor() {
    return guiFunctorGroup;
}

/*
 * Filament creation functor
 */

functor_cell_filamentCreation::functor_cell_filamentCreation(
        sGlobalVars &iGlobals
) :
        functor_cell_base(iGlobals, "Filaments", "Forces"),
        randomReal(globals.rndC->register_random("uniform_real_distribution", 0.1, 1)),
        maxCount(guiGroup->register_setting<unsigned>("Count", true, 1, 1000, 100)),
        randomAngle(guiGroup->register_setting<bool>("Random Angle", true, false)),
        maxTMV(guiGroup->register_setting<double>("TMV", true, 0, 0.1, 0.1)),
        constTMV(guiGroup->register_setting<bool>("Const. TMV", true, true)),
        maxLength(guiGroup->register_setting<double>("Length", true, 1, 500, 100)),
        infLength(guiGroup->register_setting<bool>("Inf Length", true, true)),
        maxLifeTime(guiGroup->register_setting<double>("Life Time", true, 0, 1000, 500)),
        infLifeTime(guiGroup->register_setting<bool>("Inf Life Time", true, true)),
        maxStallingForce(guiGroup->register_setting<double>("Stalling Force", true, 0, 20, 10)),
        bound1StokesCoeff(guiGroup->register_setting<double>("Min Stokes C", true, 1.0, 1000.0, 1.0)),
        bound2StokesCoeff(guiGroup->register_setting<double>("Max Stokes C", true, 1.0, 1000.0, 1.0)),
        constStokesCoeff(guiGroup->register_setting<bool>("Const Stokes C", true, true)) {}

functor_cell_filamentCreation::~functor_cell_filamentCreation() {
    globals.rndC->unregister_random(randomReal);
}

/*
 * creates initial condititon
 */

void functor_cell_filamentCreation::setup(cell &iCell) {
    for (unsigned long long i = 0; i < maxCount; i++) {
        create_filament(iCell);
    }
}

/*
 * do a single time step
 */

void functor_cell_filamentCreation::make_timeStep(double &dT, cell *iCell) {
    auto _diff = maxCount - iCell->get_filaments().size();
    if (_diff > 0) {
        for (unsigned _i = 0; _i < _diff; _i++) {
            create_filament(*iCell);
        }
    }
}

/*
 * creates a single filament
 */

filament_base *functor_cell_filamentCreation::create_filament(cell &iCell) {
    auto _par = find_creationParameters(iCell);
    membrane_part_base *_membrane = std::get<0>(_par);
    auto _con = new mf_linker(globals, iCell);
    auto *_newActin = new actin(
            globals,
            iCell,
            std::get<1>(_par),
            std::get<2>(_par),
            find_maxLength(iCell),
            find_lifeTime(iCell),
            find_stallingForce(iCell),
            find_stokesCoeff(iCell),
            functors
    );
    _newActin->add_connectedLinker(_con);
    _membrane->add_connectedLinker(_con);
    _con->add_connectedComponent(_newActin);
    iCell.register_filament(_newActin);
    iCell.register_linker(_con);
    return _newActin;
}

/*
 * finds creation position along the membrane
 */

std::tuple<membrane_part_base *, Eigen::Vector3d, Eigen::Vector3d>
functor_cell_filamentCreation::find_creationParameters(cell &iCell) {
    // get membrane and membrane parts
    auto &_membrane = *(iCell.get_membranes().begin());
    auto &_parts = _membrane->get_parts();
    // determine new position along membrane
    auto _length = _membrane->get_length();
    _length *= randomReal->draw<double>();
    auto _it = _parts.begin();
    auto _currLength = (*_it)->get_length();
    while (_currLength < _length) {
        _it++;
        _currLength += (*_it)->get_length();
    }
    auto &_pos = (*_it)->get_positions();
    _length -= _currLength;
    if (auto _el = dynamic_cast<arc_membrane_part *>(*_it)) {
        // calculate creation position
        double &_x = _pos[0](0);
        double &_y = _pos[0](1);
        auto &_R = _el->get_parameters()[0];
        auto &_angleB = _el->get_parameters()[1];
        auto &_angleE = _el->get_parameters()[2];
        auto _angle = _angleE + _length / _R;
        auto _spawnPos = Eigen::Vector3d(_x + _R * cos(_angle), _y + _R * sin(_angle), 0);
        // calculate tm velocity vector
        auto _normal = _el->get_normal(_angle);
        double _tmv = 0;
        if (constTMV) {
            _tmv = maxTMV;
        } else {
            _tmv = maxTMV * randomReal->draw<double>();
        }
        return {(*_it), _spawnPos, _tmv * _normal};
    } else if (auto _el = dynamic_cast<membrane_part *>(*_it)) {
        // calculate creation position
        auto _spawnPos = Eigen::Vector3d(_pos[0] + _length * (_pos[0] - _pos[1]).normalized());
        // calculate tm velocity vector
        auto &_normal = _el->get_normal();
        // determine tread milling velocity
        double _tmv = 0;
        if (constTMV) {
            _tmv = maxTMV;
        } else {
            _tmv = maxTMV * randomReal->draw<double>();
        }
        // determine spawn angle
        double _deg = 0;
        if (randomAngle) {
            _deg = PI * (randomReal->draw<double>() - 0.5);
        }
        Eigen::AngleAxis<double> _rot(_deg, Eigen::Vector3d(0, 0, 1));
        auto _tmvV = Eigen::Vector3d(_tmv * (_rot * (-1 * _normal)));
        return {(*_it), _spawnPos, _tmvV};
    } else {
        return {(*_it), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
    }
}

double functor_cell_filamentCreation::find_maxLength(cell &iCell) {
    if (infLength) {
        return std::numeric_limits<double>::infinity();
    }
    return maxLength * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_lifeTime(cell &iCell) {
    if (infLifeTime) {
        return std::numeric_limits<double>::infinity();
    }
    return maxLifeTime * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_stallingForce(cell &iCell) {
    return maxStallingForce * randomReal->draw<double>();
}

double functor_cell_filamentCreation::find_stokesCoeff(cell &iCell) {
    auto _min = std::min(bound1StokesCoeff, bound2StokesCoeff);
    if (!constStokesCoeff) {
        auto _max = std::max(bound1StokesCoeff, bound2StokesCoeff);
        return _min + randomReal->draw<double>() * (_max - _min);
    }
    return _min;
}

functor_cell_membraneCreation::functor_cell_membraneCreation(sGlobalVars &iGlobals) :
        functor_cell_base(iGlobals, "Membrane", "Forces"),
        radius(guiGroup->register_setting<double>("Radius", false, 0, 200, 100)),
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 20, 200, 20)) {
}

functor_cell_membraneCreation::~functor_cell_membraneCreation() = default;

void functor_cell_membraneCreation::setup(cell &iCell) {
    guiGroup->forceVariableUpdate();
    // create new mebrane
    auto &_membranes = iCell.get_membranes();
    auto *_newMembrane = new membrane_container(globals, iCell);
    _membranes.insert(_newMembrane);
    // get some data for membrane parts creation
    auto &_parts = _newMembrane->get_parts();
    auto _x = iCell.get_x();
    auto _y = iCell.get_y();
    _x = std::min(max(_x, radius), (double) globals.settings.sideLength);
    _y = std::min(max(_y, radius), (double) globals.settings.sideLength);
    // create membrane parts in circular shape
    auto _dAngle = 2 * PI / (double) resolution;
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        _parts.push_back(new membrane_part(
                globals,
                iCell,
                radius * cos(_i * _dAngle) + _x,
                radius * sin(_i * _dAngle) + _y,
                radius * cos((_i + 1) * _dAngle) + _x,
                radius * sin((_i + 1) * _dAngle) + _y,
                functors
        ));
    };
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &_sharedPositions = _parts[_i]->get_sharedPositions();
        membrane_part_base *_partA = _parts[(_i - 1) % resolution];
        membrane_part_base *_partB = _parts[(_i + 1) % resolution];
        _sharedPositions.first = &_partA->get_positions()[1];
        _sharedPositions.second = &_partB->get_positions()[0];
        _parts[_i]->set_neighbours({_partA, _partB});
    };
}

functor_cell_arcMembraneCreation::functor_cell_arcMembraneCreation(sGlobalVars &iGlobals) :
        functor_cell_membraneCreation(iGlobals) {}

functor_cell_arcMembraneCreation::~functor_cell_arcMembraneCreation() =
default;

void functor_cell_arcMembraneCreation::setup(cell &iCell) {
    guiGroup->forceVariableUpdate();
    // create new mebrane
    auto &_membranes = iCell.get_membranes();
    auto *_newMembrane = new membrane_container(globals, iCell);
    _membranes.insert(_newMembrane);
    // get some data for membrane parts creation
    auto &_parts = _newMembrane->get_parts();
    auto _x = iCell.get_x();
    auto _y = iCell.get_y();
    _x = std::min(max(_x, radius), (double) globals.settings.sideLength);
    _y = std::min(max(_y, radius), (double) globals.settings.sideLength);
    // create membrane parts
    auto _dAngle = 2 * PI / (double) resolution;
    auto _oAngle = asin(1.5d * sin(0.5d * _dAngle)) - 0.5d * _dAngle;
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        auto _cAngle = (_i + 0.5) * _dAngle;
        auto _bAngle = M_PI + _cAngle - _oAngle;
        auto _eAngle = M_PI + _cAngle + _oAngle;
        _parts.push_back(new arc_membrane_part(
                globals,
                iCell,
                3 * radius * cos(_cAngle) + _x,
                3 * radius * sin(_cAngle) + _y,
                2 * radius,
                _bAngle,
                _eAngle,
                functors
        ));
    };
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &_sharedPositions = _parts[_i]->get_sharedPositions();
        membrane_part_base *_partA = _parts[(_i - 1) % resolution];
        membrane_part_base *_partB = _parts[(_i + 1) % resolution];
        _sharedPositions.first = &_partA->get_positions()[1];
        _sharedPositions.second = &_partB->get_positions()[0];
        _parts[_i]->set_neighbours({_partA, _partB});
    };
}
