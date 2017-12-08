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

cell_base::cell_base(sGlobalVars &iGlobals) : components_base(iGlobals) {
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
    //globals.grid->register_component(this);
};

linker_base::~linker_base() {
    /*for (auto _it : connectedComponents) {
        if (_it) {
            _it->remove_connectedLinker(this);
        }
    }
    cell.unregister_linker(this);*/
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
    vend = new membrane_part_base(iGlobals, iCell);
    iGlobals.grid->unregister_component(vend);
    vend->set_next(nullptr);
    vend->set_next(nullptr);
    vbegin = nullptr;
    vback = nullptr;
    vsize = 0;
};

membrane_container::~membrane_container() {
    auto _it = begin();
    while (_it != end()) {
        auto _temp = _it->itnext();
        delete _it;
        _it = nullptr;
        _it = _temp;
    }
    delete vend;
    vbegin = nullptr;
    vback = nullptr;
    vend = nullptr;
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
    if (iPos == vend) {
        std::cout << "Couldn't insert memebrane part!\n";
        return nullptr;
    }
    if (vsize == 0) {
        iPart->set_next(iPart);
        iPart->set_prev(iPart);
        vbegin = iPart;
        vback = iPart;
    } else {
        auto _prev = iPos->prev();
        _prev->set_next(iPart);
        iPos->set_prev(iPart);
        iPart->set_prev(_prev);
        iPart->set_next(iPos);
        if (iPos == vbegin) {
            vbegin = iPart;
        }
    }
    vsize++;
    return iPart;
};

membrane_part_base *membrane_container::insert_after(membrane_part_base *iPos, membrane_part_base *iPart) {
    if (iPos == vend) {
        std::cout << "Couldn't insert memebrane part!\n";
        return nullptr;
    }
    if (vsize == 0) {
        iPart->set_next(iPart);
        iPart->set_prev(iPart);
        vbegin = iPart;
        vback = iPart;
    } else {
        auto _next = iPos->next();
        _next->set_prev(iPart);
        iPos->set_next(iPart);
        iPart->set_prev(iPos);
        iPart->set_next(_next);
        if (iPos == vback) {
            vback = iPart;
        }
    }
    vsize++;
    return iPart;
};

membrane_part_base *membrane_container::delete_part(membrane_part_base *iPart) {
    if (vsize == 0) {
        return nullptr;
    }
    auto _pneigh = iPart->prev();
    auto _nneigh = iPart->next();
    _pneigh->set_next(_nneigh);
    _nneigh->set_prev(_pneigh);
    delete iPart;
    iPart = nullptr;


    if (vsize == 1) {
        vbegin = nullptr;
        vback = nullptr;
    }
    vsize--;
    return _pneigh;
}

membrane_part_base *membrane_container::back() {
    return vback;
}

membrane_part_base *membrane_container::begin() {
    return vbegin;
}

membrane_part_base *membrane_container::end() {
    return vend;
}

unsigned long long membrane_container::size() {
    return vsize;
}

/***************************
 * Membrane Part Base
 ***************************/

membrane_part_base::membrane_part_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        cellcomponents_base(iGlobals, iCell) {
    globals.grid->register_component(this);
}

membrane_part_base::~membrane_part_base() {
    globals.grid->unregister_component(this);
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
        sGlobalVars &iGlobals,
        std::string iName,
        std::string iFunctorGroupName
) : functor_cell_base(iGlobals, iName, iFunctorGroupName) {

}

functor_membrane_base::~functor_membrane_base() = default;

void functor_membrane_base::make_timeStep(double &dT, membrane_part_base *iMembrane) {
    /* do nothing */
}

membrane_part_base *functor_membrane_base::split(
        cell_base *iCell,
        membrane_part_base *iMembranePart,
        const Eigen::Vector3d &iPos,
        linker_base *iLinker
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

