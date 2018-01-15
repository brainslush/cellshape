//
// Created by brainslush on 26/11/17.
//

#include "cMembraneFunctor.h"


/*************************************
* Membrane creation functor
*************************************/

functor_cell_membraneCreation::functor_cell_membraneCreation(sGlobalVars &iGlobals) :
        functor_membrane_base(iGlobals),
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 4, 200, 20)) {
    updateLength = true;
}

functor_cell_membraneCreation::~functor_cell_membraneCreation() = default;

void functor_cell_membraneCreation::setup(cell &iCell) {
    guiGroup->forceVariableUpdate();
    // create new mebrane
    /*if (iCell.get_membrane()) {
        delete iCell.get_membrane();
    };*/
    auto _membrane = new membrane_container(globals, iCell);
    iCell.set_membrane(_membrane);
    // get some data for membrane parts creation
    auto _x = iCell.get_x();
    auto _y = iCell.get_y();
    auto _realRadius = get_realRadius();
    _x = min(max(_x, _realRadius), (double) globals.settings->sideLength);
    _y = min(max(_y, _realRadius), (double) globals.settings->sideLength);
    // create membrane parts in circular shape
    auto _dAngle = 2 * PI / (double) resolution;
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        _membrane->insert_after(_membrane->back(), new membrane_part(
                globals,
                iCell,
                _realRadius * cos(_i * _dAngle) + _x,
                _realRadius * sin(_i * _dAngle) + _y,
                _realRadius * cos((_i + 1) * _dAngle) + _x,
                _realRadius * sin((_i + 1) * _dAngle) + _y,
                functors
        ));
    };
    auto _it = _membrane->begin();
    while (_it != _membrane->end()) {
        auto _el = dynamic_cast<membrane_part *>(_it);
        _it = _it->itnext();
    }
    updateLength = true;
}

/*
 * finds the linkers two membrane parts share
 */

linker_base *functor_cell_membraneCreation::find_sharedLinker(
        membrane_part *iPartA,
        membrane_part *iPartB
) {
    auto &_linkersA = iPartA->get_connectedLinkers();
    auto &_linkersB = iPartB->get_connectedLinkers();
    std::vector<linker_base *> _ret(std::max(_linkersA.size(), _linkersB.size()));
    std::vector<linker_base *>::iterator _unregIt;
    auto _retIt = std::set_intersection(_linkersA.begin(), _linkersA.end(), _linkersB.begin(),
                                        _linkersB.end(), _ret.begin());
    _ret.resize(_retIt - _ret.begin());
    if (_ret.size() > 1) {
        std::cout << "error: in shared linker find\n";
        return nullptr;
    }
    if (_ret.empty()) {
        return nullptr;
    }
    return _ret[0];
}

/*
 * splits the membrane into two parts at the given position
 * returns the newly created memebrane
 */

membrane_linker_base *functor_cell_membraneCreation::split(
        cell_base *iCell,
        membrane_part_base *iMembranePart,
        const Eigen::Vector3d &iPos
) {
    // gather data
    auto _cell = dynamic_cast<cell *>(iCell);
    auto _membranePart = dynamic_cast<membrane_part *>(iMembranePart);
    auto _neigh = dynamic_cast<membrane_part *>(iMembranePart->prev());
    auto &_positions = iMembranePart->get_positions();

    // create new element
    auto _newMembranePart = new membrane_part(
            globals,
            *iCell,
            _positions[0],
            iPos,
            functors
    );

    // insert new element
    _cell->get_membrane()->insert_before(iMembranePart, _newMembranePart);
    _positions[0] = iPos;

    // set new element data
    //_newMembranePart->set_sharedPositions({&(_neigh->get_positions()[1]), &(_positions[0])});

    // set neighbour new data
    //_neigh->set_sharedPositions({_neigh->get_sharedPositions().first, &(_newMembranePart->get_positions()[0])});

    // set new parameters of this membrane part
    //_membranePart->get_sharedPositions().first = &(_newMembranePart->get_positions()[1]);
    updateLength = true;
    return _newMembranePart->nextLinker();
}

/*
 * finds the starting position of the membrane chain for merge
 */
membrane_part_base *functor_cell_membraneCreation::mergeChainFront(
        cell_base *iCell,
        membrane_part_base *iMembrane
) {
    if (iMembrane != iMembrane->prev()) {
        if (iMembrane->prevLinker()->connectedFillament()) {
            return iMembrane;
        } else {
            return mergeChainFront(iCell, iMembrane->prev());
        }
    } else {
        std::cout << "Can't merge, only one element";
        return nullptr;
    }

}

/*
 * finds the end position of the
 */

membrane_part_base *functor_cell_membraneCreation::mergeChainBack(
        cell_base *iCell,
        membrane_part_base *iMembrane
) {
    if (iMembrane != iMembrane->next()) {
        if (iMembrane->nextLinker()->connectedFillament()) {
            return iMembrane->prev();
        } else {
            return mergeChainBack(iCell, iMembrane->next());
        }
    } else {
        std::cout << "Can't merge, only one element";
        return nullptr;
    }
}

membrane_part_base *functor_cell_membraneCreation::mergeChain(
        cell_base *iCell,
        membrane_part_base *iMembrane
) {
    auto _back = mergeChainBack(iCell, iMembrane);
    auto _front = mergeChainFront(iCell, iMembrane);
    auto _prev = _front->prev();
    auto _next = _back->next();

    // update shared positions
    //auto &_prevSharedPos = dynamic_cast<membrane_part *>(_prev)->get_sharedPositions();
    //auto &_nextSharedPos = dynamic_cast<membrane_part *>(_next)->get_sharedPositions();
    //auto &_prevPos = _prev->get_positions();
    //auto &_nextPos = _next->get_positions();
    //_prevSharedPos.second = &_nextPos[0];
    //_nextSharedPos.first = &_prevPos[1];

    // remove membranes
    iCell->get_membrane()->check_integrity("before deletion");
    auto _itm = _front;
    while (_itm != _next) {
        _itm = iCell->get_membrane()->delete_part(_itm);
        iCell->get_membrane()->check_integrity("after one deletion");
    }
    updateLength = true;
    return _itm;
}

void functor_cell_membraneCreation::merge(cell_base *iCell) {
    auto _membrane = iCell->get_membrane();
    auto _it = _membrane->begin();
    while (_it != _membrane->end()) {
        //std::cout << _it << "\n";
        // check for linkers
        if (!_it->nextLinker()->connectedFillament() || !_it->prevLinker()->connectedFillament()) {
            _it = mergeChain(iCell, _it);
        }
        _it = _it->itnext();
    }
};

void functor_cell_membraneCreation::make_timeStep(double &dT, cell *iCell) {
    // membrane make a time step
    merge(iCell);
    update_positions(iCell);
    updateLength = true;
}


/*
 * calculate length
 * */
void functor_cell_membraneCreation::update_length(cell_base *iCell) {
    length = 0;
    auto _membrane = iCell->get_membrane();
    auto _it = _membrane->begin();
    while (_it != _membrane->end()) {
        auto &_pos = _it->get_positions();
        length += (_pos[1] - _pos[0]).norm();
        _it = _it->itnext();
    }
    updateLength = false;
}

double functor_cell_membraneCreation::get_length(cell_base *iCell) {
    if (updateLength) {
        update_length(iCell);
    }
    return length;
}

void functor_cell_membraneCreation::update_positions(cell_base *iCell) {
    auto _it = iCell->get_membrane()->begin();
    while (_it != iCell->get_membrane()->end()) {
        auto &_pos = _it->get_positions();
        _pos[0] = *_it->prevLinker()->referencePos();
        _pos[1] = *_it->nextLinker()->referencePos();
        _it = _it->itnext();
    }
}

/*************************************
* arc membrane creation functor
*************************************/

functor_cell_arcMembraneCreation::functor_cell_arcMembraneCreation(sGlobalVars &iGlobals) :
        functor_cell_membraneCreation(iGlobals) {}

functor_cell_arcMembraneCreation::~functor_cell_arcMembraneCreation() =
default;

void functor_cell_arcMembraneCreation::setup(cell &iCell) {
    guiGroup->forceVariableUpdate();
    // create new membrane
    auto _membrane = iCell.get_membrane();
    delete _membrane;
    _membrane = new membrane_container(globals, iCell);
    // get some data for membrane parts creation
    auto _x = iCell.get_x();
    auto _y = iCell.get_y();
    auto _realRadius = get_realRadius();
    _x = std::min(max(_x, _realRadius), (double) globals.settings->sideLength);
    _y = std::min(max(_y, _realRadius), (double) globals.settings->sideLength);
    // create membrane parts
    auto _dAngle = 2 * PI / (double) resolution;
    auto _oAngle = asin(1.5d * sin(0.5d * _dAngle)) - 0.5d * _dAngle;
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        auto _cAngle = (_i + 0.5) * _dAngle;
        auto _bAngle = M_PI + _cAngle - _oAngle;
        auto _eAngle = M_PI + _cAngle + _oAngle;
        auto _newMembrane = new arc_membrane_part(
                globals,
                iCell,
                3 * _realRadius * cos(_cAngle) + _x,
                3 * _realRadius * sin(_cAngle) + _y,
                2 * _realRadius,
                _bAngle,
                _eAngle,
                functors
        );
        _membrane->insert_after(_membrane->end(), _newMembrane);
    };
}

/*************************************
* hyperbolic membrane creation functor
*************************************/

functor_cell_hyperbolicMembraneCreation::functor_cell_hyperbolicMembraneCreation(sGlobalVars &iGlobals)
        : functor_cell_membraneCreation(iGlobals),
          epsilon(guiGroup->register_setting<double>("epsilon", true, 0.001, 0.49, 0.01)),
          bp(guiGroup->register_setting<double>("bp", true, 0.1 * M_PI, M_PI / 2, M_PI / 2)) {
    oldEpsilon = -1.0d;
    oldBp = -1.0d;
    update_variables();
}

functor_cell_hyperbolicMembraneCreation::~functor_cell_hyperbolicMembraneCreation() = default;

void functor_cell_hyperbolicMembraneCreation::setup(cell &iCell) {
    auto &_filaments = dynamic_cast<functor_cell_filamentCreation *>(iCell.get_filamentFunctor())->get_filamentSetupOrder();
    if (_filaments.empty()) { return void(); };
    guiGroup->forceVariableUpdate();
    update_variables();
    // create new mebrane
    auto _membrane = new membrane_container(globals, iCell);
    iCell.set_membrane(_membrane);
    // create membrane elements
    for (unsigned _i = 0; _i < _filaments.size(); _i++) {
        actin *_filamentA = _filaments.at(_i).second;
        actin *_filamentB = _filaments.at((_i + 1) % _filaments.size()).second;
        Eigen::Vector3d &_posA = _filamentA->get_positions()[0];
        Eigen::Vector3d &_posB = _filamentB->get_positions()[0];
        Eigen::Vector3d _mdirVec = (_posB - _posA);
        auto _L = _mdirVec.norm();
        _mdirVec.normalize();
        Eigen::Vector3d _dirVec = _mdirVec.cross(Eigen::Vector3d(0, 0, -1));
        _dirVec.normalize();
        auto _dir = bmath::angleVector2d(_dirVec(0), _dirVec(1));

        auto _d = _L * cutoff;
        auto _x0 = 0.5 * _L;
        auto _y0 = cotbp * _x0;
        auto _a = _L * alpha;
        auto _b = _L * beta;

        Eigen::Vector3d _pos = _posA + _x0 * _mdirVec + _y0 * _dirVec;

        auto _t2 = atanh(alpha / beta);
        auto _t1 = -_t2;

        auto _newMembrane = new hyperbolic_membrane_part(
                globals, iCell,
                _pos,
                _d,
                _a, _b,
                _t1, _t2,
                _dir,
                functors
        );

        _membrane->insert_after(_membrane->back(), _newMembrane);
        auto _prevLinker = _newMembrane->prevLinker();
        auto _nextLinker = _newMembrane->nextLinker();
        _filamentA->set_connectedMembraneLinker(_prevLinker);
        _filamentB->set_connectedMembraneLinker(_nextLinker);
        _prevLinker->set_connectedFilament(_filamentA);
        _prevLinker->set_referencePosition(&_posA);
        _nextLinker->set_connectedFilament(_filamentB);
        _nextLinker->set_referencePosition(&_posB);
    }
}

/*
 * returns all the membrane peaks
 * it is a brute force method but with not too many membrane parts
 * it should be fast enough
 */

std::vector<membrane_linker_base *> functor_cell_hyperbolicMembraneCreation::membranepeaks(cell &iCell) {
    // find membrane with largest x position
    auto _it = iCell.get_membrane()->begin();
    auto _startLinker = _it->nextLinker();
    auto &_refPos = *_startLinker->referencePos();
    while (_it != iCell.get_membrane()->back()) {
        _it = _it->next();
        auto &_nextPos = *_it->nextLinker()->referencePos();
        if (_refPos(0) < _nextPos(0)) {
            _refPos = _nextPos;
            _startLinker = _it->nextLinker();
        }
    }
    // find all the peaks
    std::vector<membrane_linker_base *> _peaks;
    std::set<membrane_linker_base *> _tracker;
    _peaks.push_back(_startLinker);
    _tracker.insert(_startLinker);
    auto _endL = _startLinker;
    auto _itL = _startLinker->nextMembrane()->nextLinker();
    while (_itL != _endL) {
        auto _refVec = *_itL->referencePos() - *_startLinker->referencePos();
        auto _itL2 = _itL;
        auto _nextPeak = _itL;
        auto _refAngle = 0.0d;
        while (_itL2 != _endL) {
            _itL2 = _itL2->nextMembrane()->nextLinker();
            auto _vec = *_itL2->referencePos() - *_itL->referencePos();
            auto _angle = bmath::angleVector2d(_refVec(0), _refVec(1), _vec(0), _vec(1));
            if (_angle > M_PI) {
                _angle -= M_PI_2;
            }
            if (_angle > M_PI) {
                _itL2 = _endL;
            }
            if (_angle >= _refAngle) {
                _nextPeak = _itL2;
                _refAngle = _angle;
            }
        }
        _peaks.push_back(_nextPeak);
        _itL = _nextPeak;
    }
    return _peaks;
}

void functor_cell_hyperbolicMembraneCreation::update_positions(cell_base *iCell) {
    //iCell->get_filamentFunctor();
}

void functor_cell_hyperbolicMembraneCreation::make_timeStep(double &dT, cell *iCell) {
    functor_cell_membraneCreation::make_timeStep(dT, iCell);
}

void functor_cell_hyperbolicMembraneCreation::update_variables() {
    if (oldBp != bp || oldEpsilon != epsilon) {
        cotbp = 1 / tan(bp / 2.0d);
        cutoff = cotbp * (0.5d + epsilon);
        alpha2 = epsilon * epsilon + epsilon;
        alpha = sqrt(alpha2);
        beta2 = cotbp * cotbp * alpha2;
        beta = cotbp * alpha;
        oldBp = bp;
        oldEpsilon = epsilon;
    }
}
