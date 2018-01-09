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
        randomReal(globals.rndC->register_random("uniform_real_distribution", 0, 1)),
        maxCount(guiGroup->register_setting<unsigned>("Count", true, 10, 100, 40)),
        randomAngle(guiGroup->register_setting<bool>("Random Angle", true, false)),
        maxTMV(guiGroup->register_setting<double>("TMV", true, 0, 0.1, 0.01)),
        constTMV(guiGroup->register_setting<bool>("Const. TMV", true, true)),
        maxLength(guiGroup->register_setting<double>("Length", true, 0.01, 1, 0.5)),
        infLength(guiGroup->register_setting<bool>("Inf Length", true, true)),
        maxLifeTime(guiGroup->register_setting<double>("Life Time", true, 0, 1000, 500)),
        infLifeTime(guiGroup->register_setting<bool>("Inf Life Time", true, true)),
        //maxStallingForce(guiGroup->register_setting<double>("Stalling Force", true, 0, 20, 10)),
        stokesCoeff(guiGroup->register_setting<double>("Min Stokes C", true, 0.1, 10.0, 1.0)) {}
        //bound1StokesCoeff(guiGroup->register_setting<double>("Min Stokes C", true, 1.0, 1000.0, 1.0))
        //bound2StokesCoeff(guiGroup->register_setting<double>("Max Stokes C", true, 1.0, 1000.0, 1.0)),
        //constStokesCoeff(guiGroup->register_setting<bool>("Const Stokes C", true, true)) {}

functor_cell_filamentCreation::~functor_cell_filamentCreation() {
    globals.rndC->unregister_random(randomReal);
}

/*
 * creates initial condititon
 */

void functor_cell_filamentCreation::setup(cell &iCell) {
    filamentSetupOrder.clear();
    for (unsigned long long i = 0; i < maxCount; i++) {
        create_filament(iCell);
    }
    std::sort(filamentSetupOrder.begin(), filamentSetupOrder.end() /*bmath::sortpairbysec*/);
    if (auto _mfunctor = dynamic_cast<functor_cell_hyperbolicMembraneCreation *>(iCell.get_membraneFunctor())) {
        _mfunctor->setup(iCell);
    } else {
        iCell.get_membraneFunctor()->merge(&iCell);
        iCell.get_membraneFunctor()->update_positions(&iCell);
    }
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
        _filament->get_connectedMembraneLinker()->clear_filament();
        _filament->set_connectedMembraneLinker(nullptr);
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

    // create new filament
    auto *_newActin = new actin(
            globals,
            iCell,
            _pos,
            std::get<2>(_par),
            find_maxLength(iCell),
            find_lifeTime(iCell),
            //find_stallingForce(iCell),
            //find_stokesCoeff(iCell),
            stokesCoeff,
            functors
    );

    // check if membrane was made
    if (_membrane) {
        // create new membrane
        auto _linker = iCell.get_membraneFunctor()->split(&iCell, _membrane, _pos);
        // register linker at the actin
        _newActin->set_connectedMembraneLinker(_linker);
        // register actin at the linker
        _linker->set_referencePosition(&_newActin->get_positions()[1]);
        _linker->set_connectedFilament(_newActin);
        // register actin at the cell
    } else {
        filamentSetupOrder.back().second = _newActin;
    }
    iCell.register_filament(_newActin);
    return _newActin;
}


/*
 * finds creation position along the membrane
 * returns the new membrane and the position
 */

std::tuple<membrane_part_base *, Eigen::Vector3d, Eigen::Vector3d>
functor_cell_filamentCreation::find_creationParameters(cell &iCell) {
    // get membrane and membrane parts
    auto _membrane = iCell.get_membrane();
    // determine threadmiling velocity
    auto _tmv = find_tmv(iCell);
    // handle case where the membrane is created after the filaments for simplicity
    if (_membrane) {
        // determine new position along membrane
        auto _length = iCell.get_membraneFunctor()->get_length(&iCell);
        _length *= randomReal->draw<double>();
        auto _it = _membrane->begin();
        auto _currLength = _it->get_length();
        auto _itLength = _it->get_length();
        while (_currLength + _itLength < _length && _it != _membrane->end()) {
            _currLength += _itLength;
            _it = _it->next();
            _itLength = _it->get_length();
        }
        if (_it == _membrane->end()) {
            std::cout << "Error(membrane split): reached membrane container end\n";
        }
        auto &_pos = _it->get_positions();
        _length -= _currLength;
        if (_length < 0) {
            std::cout << "Error(membrane split): negative length\n";
        }
        // create filament
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
            return {_it, _spawnPos, _tmv * _normal};
        } else if (auto _el = dynamic_cast<membrane_part *>(_it)) {
            // calculate creation position
            auto _spawnPos = Eigen::Vector3d(_pos[0] + _length * (_pos[1] - _pos[0]).normalized());
            // calculate tm velocity vector
            auto &_normal = _el->get_normal();
            // determine spawn angle
            double _deg = 0;
            if (randomAngle) {
                _deg = PI * (randomReal->draw<double>() - 0.5);
            }
            Eigen::AngleAxis<double> _rot(_deg, Eigen::Vector3d(0, 0, 1));
            auto _tmvV = Eigen::Vector3d(_tmv * (_rot * (-1 * _normal)));
            return {_it, _spawnPos, _tmvV};
        } else if (auto _el = dynamic_cast<hyperbolic_membrane_part *>(_it)) {
            // get hyperbola segment
            auto _idl = _el->get_segmentId(_currLength);
            auto _seg0 = _el->get_segment(_idl.first);
            auto _seg1 = _el->get_segment(_idl.first + 1);
            // calculate creation position
            auto _spawnPos = Eigen::Vector3d(_seg0 + _length * (_pos[0] - _pos[1]).normalized());
            // velocity vector
            auto _normal = _el->get_normal(_idl.first);
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
    } else {
        auto _membraneFunctor = iCell.get_membraneFunctor();
        // determine spawn position
        auto _spawnPos = Eigen::Vector3d(iCell.get_x(), iCell.get_y(), 0);
        auto _realRadius = _membraneFunctor->get_realRadius();
        auto _posAngle = M_PI_2 * randomReal->draw<double>();
        auto _radial = Eigen::Vector3d(_realRadius * cos(_posAngle), _realRadius * sin(_posAngle), 0);
        _spawnPos += _radial;
        // determine spawn angle
        double _deg = 0;
        if (randomAngle) {
            _deg = PI * (randomReal->draw<double>() - 0.5);
        }
        Eigen::AngleAxis<double> _rot(_deg, Eigen::Vector3d(0, 0, 1));
        auto _normal = -_radial.normalized();
        auto _tmvV = Eigen::Vector3d(_tmv * (_rot * (-1 * _normal)));
        filamentSetupOrder.emplace_back(make_pair(_posAngle, nullptr));
        return {nullptr, _spawnPos, _tmvV};
    }
}

double functor_cell_filamentCreation::find_maxLength(cell &iCell) {
    if (infLength) {
        return std::numeric_limits<double>::infinity();
    }
    return maxLength * (randomReal->draw<double>() * 0.9 + 0.1);
}

double functor_cell_filamentCreation::find_lifeTime(cell &iCell) {
    if (infLifeTime) {
        return std::numeric_limits<double>::infinity();
    }
    return maxLifeTime * (randomReal->draw<double>() * 0.9 + 0.1);
}

double functor_cell_filamentCreation::find_tmv(cell &iCell) {
    if (constTMV) {
        return maxTMV;
    } else {
        return maxTMV * (randomReal->draw<double>() * 0.9 + 0.1);
    }
}

/*double functor_cell_filamentCreation::find_stallingForce(cell &iCell) {
    return maxStallingForce * randomReal->draw<double>();
}
*/

/*
const double &functor_cell_filamentCreation::find_stokesCoeff(cell &iCell) {
    auto _min = std::min(bound1StokesCoeff, bound2StokesCoeff);
    if (!constStokesCoeff) {
        auto _max = std::max(bound1StokesCoeff, bound2StokesCoeff);
        return _min + randomReal->draw<double>() * (_max - _min);
    }
    return _min;
    return stokesCoeff;
}
*/
