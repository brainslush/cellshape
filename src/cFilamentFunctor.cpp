//
// Created by brainslush on 26/11/17.
//

#include "cFilamentFunctor.h"


/*************************************
* Filament creation functor
*************************************/

functor_cell_filamentCreation::functor_cell_filamentCreation(
        sGlobalVars &iGlobals
) :
        functor_filament_base(iGlobals, "Filaments", "Forces"),
        randomReal(globals.rndC->register_random("uniform_real_distribution", 0.1, 1)),
        maxCount(guiGroup->register_setting<unsigned>("Count", true, 1, 1000, 5)),
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
    iCell.get_membraneFunctor()->merge(&iCell);
}

/*
 * do a single time step
 */

void functor_cell_filamentCreation::make_timeStep(double &dT, cell *iCell) {
    auto &_filaments = iCell->get_filaments();
    // check filament counter
    auto _diff = maxCount - _filaments.size();
    if (_diff > 0) {
        for (unsigned _i = 0; _i < _diff; _i++) {
            create_filament(*iCell);
        }
    }
    // let filaments do a single timestep
    std::vector<filament_base *> _delf;
    for (auto &_filament : _filaments) {
        if (_filament->make_timeStep(dT)) {
            _delf.push_back(_filament);
        }
    }
    // remove obselete filaments & linkers
    for (auto _filament : _delf) {
        // remove linkers
        auto &_linkers = _filament->get_connectedLinkers();
        for (auto _linker : _linkers) {
            if (dynamic_cast<mf_linker *>(_linker)) {
                iCell->get_linkerFunctor()->delete_linker(iCell, _linker);
            }
        }
        iCell->unregister_filament(_filament);
        delete _filament;
        _filament = nullptr;
    }
}

/*
 * creates a single filament
 */

filament_base *functor_cell_filamentCreation::create_filament(cell &iCell) {
    auto _par = find_creationParameters(iCell);
    auto _membrane = std::get<0>(_par);
    auto &_pos = std::get<1>(_par);

    // create new linker
    auto _linker = dynamic_cast<mf_linker *>(iCell.get_linkerFunctor()->create_linker(&iCell, {}));

    // create new membrane
    iCell.get_membraneFunctor()->split(&iCell, _membrane, _pos, _linker);

    // create new filament
    auto *_newActin = new actin(
            globals,
            iCell,
            _pos,
            std::get<2>(_par),
            find_maxLength(iCell),
            find_lifeTime(iCell),
            find_stallingForce(iCell),
            find_stokesCoeff(iCell),
            functors
    );
    // register linker at the actin
    _newActin->add_connectedLinker(_linker);
    // register actin at the linker
    _linker->set_referencePos(&_newActin->get_positions()[1]);
    _linker->set_connectedFillament(_newActin);
    // register actin at the cell
    iCell.register_filament(_newActin);
    return _newActin;
}


/*
 * finds creation position along the membrane
 */

std::tuple<membrane_part_base *, Eigen::Vector3d, Eigen::Vector3d>
functor_cell_filamentCreation::find_creationParameters(cell &iCell) {
    // get membrane and membrane parts
    auto _membrane = iCell.get_membrane();
    // determine new position along membrane
    auto _length = iCell.get_membraneFunctor()->get_length(&iCell);
    _length *= randomReal->draw<double>();
    auto _it = _membrane->begin();
    auto _currLength = _it->get_length();
    while (_currLength < _length && _it != _membrane->end()) {
        _it = _it->itnext();
        _currLength += _it->get_length();
    }
    auto &_pos = _it->get_positions();
    _length -= _currLength;
    if (auto _el = dynamic_cast<arc_membrane_part *>(_it)) {
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
        return {_it, _spawnPos, _tmv * _normal};
    } else if (auto _el = dynamic_cast<membrane_part *>(_it)) {
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
        return {_it, _spawnPos, _tmvV};
    } else {
        return {_it, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
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
