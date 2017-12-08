//
// Created by brainslush on 26/11/17.
//

#include "cMembraneFunctor.h"


/*************************************
* Membrane creation functor
*************************************/

functor_cell_membraneCreation::functor_cell_membraneCreation(sGlobalVars &iGlobals) :
        functor_membrane_base(iGlobals, "Membrane", "Forces"),
        radius(guiGroup->register_setting<double>("Radius", false, 0, 200, 100)),
        resolution(guiGroup->register_setting<unsigned>("Resolution", false, 4, 200, 4)) {
}

functor_cell_membraneCreation::~functor_cell_membraneCreation() = default;

void functor_cell_membraneCreation::setup(cell &iCell) {
    guiGroup->forceVariableUpdate();
    // create new mebrane
    auto _membrane = new membrane_container(globals, iCell);
    iCell.set_membrane(_membrane);
    // get some data for membrane parts creation
    auto _x = iCell.get_x();
    auto _y = iCell.get_y();
    _x = min(max(_x, radius), (double) globals.settings.sideLength);
    _y = min(max(_y, radius), (double) globals.settings.sideLength);
    // create membrane parts in circular shape
    auto _dAngle = 2 * PI / (double) resolution;
    for (unsigned long long _i = 0; _i < resolution; _i++) {
        _membrane->insert_after(_membrane->back(), new membrane_part(
                globals,
                iCell,
                radius * cos(_i * _dAngle) + _x,
                radius * sin(_i * _dAngle) + _y,
                radius * cos((_i + 1) * _dAngle) + _x,
                radius * sin((_i + 1) * _dAngle) + _y,
                functors
        ));
    };
    auto _it = _membrane->begin();
    while (_it != _membrane->end()) {
        auto _el = dynamic_cast<membrane_part *>(_it);
        std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &_sharedPositions = _el->get_sharedPositions();
        _sharedPositions.first = &_el->prev()->get_positions()[1];
        _sharedPositions.second = &_el->next()->get_positions()[0];
        _it = _it->itnext();
    }
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
    std::vector<linker_base *> _ret(std::max(_linkersA.size(),_linkersB.size()));
    std::vector<linker_base *>::iterator _unregIt;
    auto _retIt = std::set_intersection(_linkersA.begin(),_linkersA.end(), _linkersB.begin(),
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

membrane_part_base *functor_cell_membraneCreation::split(
        cell_base *iCell,
        membrane_part_base *iMembranePart,
        const Eigen::Vector3d &iPos,
        linker_base *iLinker
) {
    // gather data
    auto _cell = dynamic_cast<cell *>(iCell);
    auto _membranePart = dynamic_cast<membrane_part *>(iMembranePart);
    auto _neigh = dynamic_cast<membrane_part *>(iMembranePart->prev());
    auto &_positions = iMembranePart->get_positions();

    // find shared linkers
    auto _sharedL = find_sharedLinker(_membranePart, _neigh);

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
    _newMembranePart->add_connectedLinker(iLinker);
    _newMembranePart->set_sharedPositions({&(_neigh->get_positions()[1]), &(_positions[0])});

    // set neighbour new data
    _neigh->set_sharedPositions({_neigh->get_sharedPositions().first, &(_newMembranePart->get_positions()[0])});

    // set new parameters of this membrane part
    iMembranePart->add_connectedLinker(iLinker);
    iMembranePart->remove_connectedLinker(_sharedL);
    _membranePart->get_sharedPositions().first = &(_newMembranePart->get_positions()[1]);

    // update new linker information
    auto _linker = dynamic_cast<mf_linker *>(iLinker);
    auto &_fPos = _newMembranePart->get_positions();
    auto &_bPos = iMembranePart->get_positions();
    _linker->set_connectedMembranes({_newMembranePart,iMembranePart});
    _linker->set_membranePositions({&_fPos[1],&_bPos[0]});

    //update old linker information
    if (_sharedL) {
        auto _sharedLinker = dynamic_cast<mf_linker *>(_sharedL);
        _sharedLinker->set_connectedMembranes({_sharedLinker->get_connectedMembranes().first, _newMembranePart});
        _sharedLinker->set_membranePositions({_sharedLinker->get_membranePositions().first, &_fPos[0]});
    }

    return _newMembranePart;
}

/*
 * finds the starting position of the membrane chain for merge
 */
membrane_part_base *functor_cell_membraneCreation::mergeChainFront(
        cell_base *iCell,
        membrane_part_base *iMembrane
) {
    if (iMembrane != iMembrane->prev()) {
        if (iMembrane->prev()->get_connectedLinkers().empty()) {
            return mergeChainFront(iCell, iMembrane->prev());
        } else if (iMembrane->prev()->get_connectedLinkers().size() >= 2) {
            return iMembrane;
        } else {
            return iMembrane->prev();
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
        if (iMembrane->next()->get_connectedLinkers().empty()) {
            return mergeChainFront(iCell, iMembrane->next());
        } else if (iMembrane->get_connectedLinkers().size() >= 2) {
            return iMembrane;
        } else {
            return iMembrane->next();
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
    auto _end = mergeChainBack(iCell, iMembrane);
    auto _begin = mergeChainFront(iCell, iMembrane);
    auto _first = _begin->prev();

    // update shared positions
    auto &_fSharedPos = dynamic_cast<membrane_part *>(_first)->get_sharedPositions();
    auto &_eSharedPos = dynamic_cast<membrane_part *>(_end)->get_sharedPositions();
    auto &_fPos = _first->get_positions();
    auto &_ePos = _end->get_positions();
    _fSharedPos.second = &_ePos[0];
    _eSharedPos.first = &_fPos[1];

    // update linkers
    auto &_fLinker = _first->get_connectedLinkers();
    mf_linker *_linker = nullptr;
    auto _itf = _fLinker.begin();
    while (_itf != _fLinker.end()) {
        if (auto _el = dynamic_cast<mf_linker *>(*_itf)) {
            auto _fmem = _el->get_connectedMembranes().second;
            if (_fmem == _begin) {
                _linker = _el;
                _itf = _fLinker.end();
                _itf--;
            }
        }
        _itf++;
    }
    if (_linker) {
        _linker->set_connectedMembranes({_linker->get_connectedMembranes().first, _end});
        _linker->set_membranePositions({_linker->get_membranePositions().first,&_ePos[0]});
    } else {
        std::cout << "Linker is broken \n";
    }

    // remove membranes
    auto _itm = _begin;
    while (_itm != _end) {
        _itm = iCell->get_membrane()->delete_part(_itm);
    }

    return _end;
}

void functor_cell_membraneCreation::merge(cell_base *iCell) {
    auto _membrane = iCell->get_membrane();
    auto _it = _membrane->begin();
    while (_it != _membrane->end()) {
        std::cout << _it << "\n";
        // check for linkers
        auto &_linkers = _it->get_connectedLinkers();
        if (_linkers.size() < 2) {
            _it = mergeChain(iCell, _it);
        } else if (_linkers.size() > 2) {
            std::cout << "Warning (merge) : Membrane part has more than two linkers!\n";
        }
        _it = _it->itnext();
    }
};

void functor_cell_membraneCreation::make_timeStep(double &dT, cell *iCell) {
    // membrane make a time step
    merge(iCell);

}


/*
 * calculate length
 * */
void functor_cell_membraneCreation::update_length(cell_base *iCell) {
    length = 0;
    auto _cell = dynamic_cast<cell *>(iCell);
    auto _membrane = _cell->get_membrane();
    auto _it = _membrane->begin();
    while (_it != _membrane->end()) {
        auto &_pos = _it->get_positions();
        length += (_pos[1] - _pos[0]).norm();
        _it = _it->itnext();
    }
}

double functor_cell_membraneCreation::get_length(cell_base *iCell) {
    update_length(iCell);
    return length;
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
    _x = std::min(max(_x, radius), (double) globals.settings.sideLength);
    _y = std::min(max(_y, radius), (double) globals.settings.sideLength);
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
                3 * radius * cos(_cAngle) + _x,
                3 * radius * sin(_cAngle) + _y,
                2 * radius,
                _bAngle,
                _eAngle,
                functors
        );
        _membrane->insert_after(_membrane->end(), _newMembrane);
    };
}
