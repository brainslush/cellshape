#include "cBaseclasses.h"


/***************************
 * Components Base
 ***************************/

components_base::components_base(sGlobalVars &iGlobals) : base(), globals(iGlobals) {
}

components_base::~components_base() = default;

/******************************************
* base class for cell functors
*******************************************/

functor_cell_base::functor_cell_base(
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) :
        globals(iGlobals) {
    if (iName != "" && iFunctorGroupName != "") {
        guiGroup = globals.guiMain->register_group(std::move(iName));
        guiFunctorGroup = globals.guiC->register_gui(std::move(iFunctorGroupName));
    }
}

functor_cell_base::~functor_cell_base() = default;

std::set<stokes::functor *> &functor_cell_base::get_functors() {
    return functors;
};

void functor_cell_base::register_functor(stokes::functor *iFunctor) {
    functors.insert(iFunctor);
}

mygui::gui *&functor_cell_base::get_guiFunctor() {
    return guiFunctorGroup;
}

/***************************
 * Cell Base
 ***************************/

cell_base::cell_base(sGlobalVars &iGlobals) :
        components_base(iGlobals),
        membrane(nullptr)
{
}

cell_base::~cell_base() = default;

/***************************
 * cellcomponents_base
 ***************************/

cellcomponents_base::cellcomponents_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        components_base(iGlobals),
        stokes::Base::Base(),
        cell(iCell) {
}

cellcomponents_base::~cellcomponents_base() {};

std::set<linker_base *> &cellcomponents_base::get_connectedLinkers() {
    return connectedLinkers;
}

void cellcomponents_base::add_connectedLinker(linker_base *iComponent) {
    connectedLinkers.insert(iComponent);
};

void cellcomponents_base::remove_connectedLinker(linker_base *iComponent) {
    connectedLinkers.erase(iComponent);
}

/***************************
 * Linker base
 ***************************/

linker_base::linker_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        components_base(iGlobals),
        cell(iCell) {
};

linker_base::~linker_base() {
};

std::set<cellcomponents_base *> &linker_base::get_connectedComponents() {
    return connectedComponents;
}

void linker_base::add_connectedComponent(cellcomponents_base *iComponent) {
    connectedComponents.insert(iComponent);
};

void linker_base::remove_connectedComponent(cellcomponents_base *iComponent) {
    connectedComponents.erase(iComponent);
}

void linker_base::make_timeStep(const double &dT) {
    // do nothing
}

void linker_base::set_connectedComponents(const std::set<cellcomponents_base *> &iComponents) {
    connectedComponents = iComponents;
};

/***************************
 * filament base
 ***************************/

filament_base::filament_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) : cellcomponents_base(iGlobals, iCell) {
    associatedVisualObj = new visual_line(this);
    globals.grid->register_component(this);
}

filament_base::~filament_base() {
    globals.grid->unregister_component(this);
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void filament_base::set_positions(double iX1, double iY1, double iX2, double iY2) {
    positions[0](0) = iX1;
    positions[0](1) = iY1;
    positions[1](0) = iX2;
    positions[1](1) = iY2;
};

void filament_base::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
};

bool filament_base::make_timeStep(double &iTime) {
    return false;
}

membrane_linker_base *filament_base::get_connectedMembraneLinker() {
    return connectedMembraneLinker;
}

void filament_base::set_connectedMembraneLinker(membrane_linker_base *iLinker) {
    connectedMembraneLinker = iLinker;
};

/***************************
 * volume base
 ***************************/

volume_base::volume_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        cellcomponents_base(iGlobals, iCell) {
};

volume_base::~volume_base() = default;

void volume_base::make_timeStep(double &dT) {}

/***************************
 * membrane container
 ***************************/

// circular membrane
membrane_container::membrane_container(
        sGlobalVars &iGlobals,
        cell_base &iCell
) : cellcomponents_base(iGlobals, iCell) {
    vEnd = new membrane_part_base(iGlobals, iCell);
    iGlobals.grid->unregister_component(vEnd);
    vEnd->set_next(nullptr);
    vEnd->set_next(nullptr);
    vBegin = nullptr;
    vBack = nullptr;
    vSize = 0;
};

membrane_container::~membrane_container() {
    auto _it = begin();
    while (_it != end()) {
        auto _temp = _it->itnext();
        delete _it;
        _it = nullptr;
        _it = _temp;
    }
    delete vEnd;
    vBegin = nullptr;
    vBack = nullptr;
    vEnd = nullptr;
}

/* */
void membrane_container::obtain_visualObjs(std::vector<visual_base *> &oVisualComponents) {
    auto _it = begin();
    while (_it != end()) {
        _it->obtain_visualObjs(oVisualComponents);
        _it = _it->itnext();
    }
}

membrane_part_base *membrane_container::insert_before(membrane_part_base *iPos, membrane_part_base *iPart) {
    if (iPos == vEnd) {
        std::cout << "Couldn't insert memebrane part!\n";
        return nullptr;
    }
    if (vSize == 0) {
        iPart->set_next(iPart);
        iPart->set_prev(iPart);
        vBegin = iPart;
        vBack = iPart;
        auto _linker = iPart->createLinker();
        _linker->set_nextMembrane(iPart);
        _linker->set_prevMembrane(iPart);
        iPart->set_nextLinker(_linker);
        iPart->set_prevLinker(_linker);
    } else {
        auto _prev = iPos->prev();
        auto _newLinker = iPart->createLinker();

        _prev->set_next(iPart);
        iPos->set_prev(iPart);
        iPart->set_prev(_prev);
        iPart->set_next(iPos);

        // update linkers
        _newLinker->set_prevMembrane(iPart);
        _newLinker->set_nextMembrane(iPos);

        iPos->prevLinker()->set_nextMembrane(iPart);
        iPart->set_nextLinker(_newLinker);
        iPart->set_prevLinker(iPos->prevLinker());
        iPos->set_prevLinker(_newLinker);

        if (iPos == vBegin) {
            vBegin = iPart;
        }
    }
    vSize++;
    return iPart;
};

membrane_part_base *membrane_container::insert_after(membrane_part_base *iPos, membrane_part_base *iPart) {
    if (iPos == vEnd) {
        std::cout << "Couldn't insert memebrane part!\n";
        return nullptr;
    }
    if (vSize == 0) {
        iPart->set_next(iPart);
        iPart->set_prev(iPart);
        vBegin = iPart;
        vBack = iPart;
        auto _linker = iPart->createLinker();
        _linker->set_nextMembrane(iPart);
        _linker->set_prevMembrane(iPart);
        iPart->set_nextLinker(_linker);
        iPart->set_prevLinker(_linker);
    } else {
        auto _next = iPos->next();
        auto _newLinker = iPart->createLinker();

        _next->set_prev(iPart);
        iPos->set_next(iPart);
        iPart->set_prev(iPos);
        iPart->set_next(_next);

        // update linkers
        _newLinker->set_prevMembrane(iPos);
        _newLinker->set_nextMembrane(iPart);
        iPos->nextLinker()->set_prevMembrane(iPart);
        iPart->set_nextLinker(iPos->nextLinker());
        iPart->set_prevLinker(_newLinker);
        iPos->set_nextLinker(_newLinker);

        if (iPos == vBack) {
            vBack = iPart;
        }
    }
    vSize++;
    return iPart;
};

membrane_part_base *membrane_container::delete_part(membrane_part_base *iPart) {
    // check if deletion conditions are meet
    if (vSize == 0) {
        std::cout << "Couldn't delete membrane, membrane has no elements!\n";
        return nullptr;
    }
    auto _nLinker = iPart->nextLinker();
    auto _pLinker = iPart->prevLinker();

    if (iPart == vBegin) {
        vBegin = iPart->next();
    }
    if (iPart == vBack) {
        vBack = iPart->prev();
    }

    if (_nLinker->connectedFillament() && _pLinker->connectedFillament()) {
        std::cout << "Couldn't delete membrane, both linkers still carry filaments!\n";
        return nullptr;
    }
    // reasign neighbours and linkers
    auto _pneigh = iPart->prev();
    auto _nneigh = iPart->next();
    _pneigh->set_next(_nneigh);
    _nneigh->set_prev(_pneigh);
    if (!_nLinker->connectedFillament()) {
        _pLinker->set_nextMembrane(_nneigh);
        _nneigh->set_prevLinker(_pLinker);
        delete _nLinker;
        _nLinker = nullptr;
    }
    if (!_pLinker->connectedFillament() && _nLinker->connectedFillament()) {
        _nLinker->set_prevMembrane(_pneigh);
        _pneigh->set_nextLinker(_nLinker);
        delete _pLinker;
        _pLinker = nullptr;
    }

    delete iPart;
    iPart = nullptr;

    if (vSize == 1) {
        vBegin = nullptr;
        vBack = nullptr;
        if (_nLinker) {
            delete _nLinker;
            _nLinker = nullptr;
        }
        if (_pLinker) {
            delete _pLinker;
            _pLinker = nullptr;
        }
    }
    vSize--;
    return _nneigh;
}

void membrane_container::check_integrity(const std::string &iModule) {
    std::cout << "========================\n";
    std::cout << "Module: " << iModule << "\n";
    std::cout << "Start integrity check:\n";
    if (vSize == 0) {
        std::cout << "chain lenght is zero\n";
        return void();
    }
    if (vBack->itnext() != vEnd) {
        std::cout << "distached End\n";
        return void();
    }
    std::vector<membrane_part_base *> _loop;
    std::set<membrane_part_base *> _loopCheck;
    auto _it = vBegin;
    auto _end = vBack;
    unsigned _size = 0;
    unsigned _linkercount = 0;
    unsigned _emptylinkerCount = 0;
    while (_loopCheck.insert(_it).second) {
        _loop.push_back(_it);
        _it = _it->itnext();
    }
    if (_loop.size() - 1 != vSize || _loop.back() != vEnd) {
        std::cout << "Early loop detected\n";
        for (auto _el : _loop) {
            std::cout << _el << "\n";
        }
        return void();
    }
    _it = vBegin;
    while (_it != _end && _it != vEnd) {
        _size++;
        _linkercount++;
        if (_it != _it->prev()->next() || _it != _it->next()->prev()) {
            std::cout << "chain not closed @ " << _it << "\n";
            _it = _end;
        }
        if (_it != _it->prevLinker()->nextMembrane()) {
            std::cout << "Linker " << _it->prevLinker() << " broken @ " << _it << "\n";
            _it = _end;
        }
        if (_it != _it->nextLinker()->prevMembrane()) {
            std::cout << "Linker " << _it->nextLinker() << " broken @ " << _it << "\n";
            _it = _end;
        }
        if (!_it->nextLinker()->connectedFillament()) {
            std::cout << "Linker " << _it->nextLinker() << " is empty\n";
            _emptylinkerCount++;
        }

        _it = _it->itnext();
        if (_it == vBack) {
            _end = vBegin;
        }
    }
    if (_it == vEnd) {
        _it = vBack;
        std::cout << "End was reached\n";
    }
    if (_size != vSize) {
        std::cout << "size mismatch\n";
    }
    std::cout << "vSize: " << vSize << ", measured Size: " << _size << "\n";
    if (_emptylinkerCount > 0) {
        std::cout << _emptylinkerCount << " empty linkers found\n";
    }
    std::cout << "========================\n";
}

/***************************
 * Membrane Part Base
 ***************************/

membrane_part_base::membrane_part_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        cellcomponents_base(iGlobals, iCell),
        vNextLinker(nullptr),
        vPrevLinker(nullptr),
        vNext(nullptr),
        vPrev(nullptr) {
    globals.grid->register_component(this);

}

membrane_part_base::~membrane_part_base() {
    globals.grid->unregister_component(this);
    /*if (vNextLinker) {
        delete vNextLinker;
        vNextLinker = nullptr;
    }
    if (vPrevLinker) {
        delete vPrevLinker;
        vPrevLinker = nullptr;
    }*/
};

double membrane_part_base::get_length() {
    return 0;
}

/***************************
 * Matrix Compoents Base
 ***************************/

matrixcomponents_base::matrixcomponents_base(sGlobalVars &iGlobals) : components_base(iGlobals) {
}

matrixcomponents_base::~matrixcomponents_base() = default;

/***************************
 * Fac Base
 ***************************/

fac_base::fac_base(sGlobalVars &iGlobals) : matrixcomponents_base(iGlobals) {

}

fac_base::~fac_base() = default;

/***************************
 * Surface Border Base
 ***************************/

surface_border_base::surface_border_base(sGlobalVars &iGlobals) : matrixcomponents_base(iGlobals) {

}

surface_border_base::~surface_border_base() = default;

/***************************
 * Surface Base
 ***************************/

surface_base::surface_base(sGlobalVars &iGlobals) : matrixcomponents_base(iGlobals) {

}

surface_base::~surface_base() = default;

/***************************
 * Membrane Functor Base
 ***************************/

functor_membrane_base::functor_membrane_base(
        sGlobalVars &iGlobals
) : functor_cell_base(iGlobals, "Membrane", "Settings"),
    radius(guiGroup->register_setting<double>("Radius", false, 0, 200, 100)) {
}

functor_membrane_base::functor_membrane_base(
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) : functor_cell_base(iGlobals, iName, iFunctorGroupName),
    radius(guiGroup->register_setting<double>("Radius", false, 0, 200, 100)) {
}

functor_membrane_base::~functor_membrane_base() = default;

void functor_membrane_base::make_timeStep(double &dT, cell_base *iCell) {
    /* do nothing */
}

membrane_linker_base *functor_membrane_base::split(
        cell_base *iCell,
        membrane_part_base *iMembranePart,
        const Eigen::Vector3d &iPos
) {
    return nullptr;
}

void functor_membrane_base::merge(
        cell_base *iCell
) {
    /* do nothing */
}

double functor_membrane_base::get_length(cell_base *iCell) {
    return 0;
}

void functor_membrane_base::update_positions(cell_base *iCell) {

}

/***************************
 * Filament Functor Base
 ***************************/

functor_filament_base::functor_filament_base(
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) : functor_cell_base(iGlobals, iName, iFunctorGroupName) {

}

functor_filament_base::~functor_filament_base() = default;

functor_linker_base::functor_linker_base(
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) : functor_cell_base(iGlobals, std::move(iName), std::move(iFunctorGroupName)) {

}

functor_linker_base::~functor_linker_base() {

}

linker_base *functor_linker_base::create_linker(
        cell_base *iCell,
        const std::set<cellcomponents_base *> &iConnectedComponents
) {
    return nullptr;
}

void functor_linker_base::delete_linker(cell_base *iCell, linker_base *iLinker) {

}

/***************************
 * membrane linker base
 ***************************/

membrane_linker_base::membrane_linker_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        components_base(iGlobals),
        cell(iCell),
        vNextMembrane(nullptr),
        vPrevMembrane(nullptr),
        vConnectedFilament(nullptr),
        vReferencePos(nullptr) {
    associatedVisualObj = new visual_ellipse(this);
    associatedVisualObj->set_color(1, 0, 0);
    associatedVisualObj->set_fillColor(1, 0, 0);
    parameters.push_back(2);
    parameters.push_back(2);
    positions.emplace_back(Eigen::Vector3d(0, 0, 0));
}

membrane_linker_base::~membrane_linker_base() {
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void membrane_linker_base::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

void membrane_linker_base::make_timeStep(const double &dT) {
    positions[0] = *vReferencePos;
    // *vNextMembranePosition = *vReferencePos;
    // *vPrevMembranePosition = *vReferencePos;
}
