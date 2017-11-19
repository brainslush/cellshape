#include "cBaseclasses.h"


/***************************
 * Components Base
 ***************************/

components_base::components_base(sGlobalVars &iGlobals) : base(), globals(iGlobals) {
    canMove = true;
    canColide = true;
}

components_base::~components_base() = default;

bool &components_base::get_canMove() { return canMove; }

bool &components_base::get_canColide() { return canColide; }

Eigen::Vector3d &components_base::get_responseForce() { return responseForce; }

Eigen::Vector3d &components_base::get_responseTorque() { return responseTorque; }

void components_base::set_canMove(bool iCanMove) { canMove = iCanMove; }

void components_base::set_canColide(bool iCanColide) { canColide = iCanColide; }

void components_base::add_responseForce(const Eigen::Vector3d &iForce) { responseForce += iForce; }

void components_base::add_responseTorque(const Eigen::Vector3d &iTorque) { responseTorque += iTorque; }

/***************************
 * Cell Base
 ***************************/

cell_base::cell_base(sGlobalVars &iGlobals) : components_base(iGlobals) {
    canMove = true;
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

cellcomponents_base::~cellcomponents_base() = default;

/***************************
 * Linker base
 ***************************/

linker_base::linker_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        cellcomponents_base(iGlobals, iCell) {
    canColide = false;
    canMove = true;
    globals.grid->register_component(this);
};

linker_base::~linker_base() {
    globals.grid->unregister_component(this);
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
};

/***************************
 * filament base
 ***************************/

filament_base::filament_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) : cellcomponents_base(iGlobals, iCell) {
    canColide = true;
    canMove = true;
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
}

void filament_base::add_connectedLinker(linker_base *iLinker) {
    connectedLinkers.insert(iLinker);
}

void filament_base::remove_connectedLinker(linker_base *iLinker) {
    connectedLinkers.erase(iLinker);
}

void filament_base::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

bool filament_base::make_timeStep(double &iTime) {
    return false;
}

std::set<linker_base *> &filament_base::get_connectedLinkers() {
    return connectedLinkers;
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
 * Membrane Part Base
 ***************************/

membrane_part_base::membrane_part_base(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        double iX1, double iY1,
        double iX2, double iY2
) :
        cellcomponents_base(iGlobals, iCell) {
    positions.clear();
    positions.emplace_back(Eigen::Vector3d(iX1, iY1, 0));
    positions.emplace_back(Eigen::Vector3d(iX2, iY2, 0));
    associatedVisualObj = new visual_line(this);
    associatedVisualObj->set_color(0.0, 0.0, 0.0);
    associatedVisualObj->set_fillColor(0.0, 0.0, 0.0);
    globals.grid->register_component(this);
}

membrane_part_base::~membrane_part_base() {
    globals.grid->unregister_component(this);
};

std::pair<membrane_part_base *, membrane_part_base *> &membrane_part_base::get_neighbours() {
    return neighbours;
}

double membrane_part_base::get_length() {
    return (positions[1] - positions[0]).norm();
}

std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &membrane_part_base::get_sharedPositions() {
    return sharedPositions;
}

void membrane_part_base::set_neighbours(const std::pair<membrane_part_base *, membrane_part_base *> &iNeighbours) {
    neighbours = iNeighbours;
}

Eigen::Vector3d membrane_part_base::calc_dirVector(Eigen::Vector3d *iPoint) {
    if (iPoint == &positions[0]) {
        return (positions[0] - positions[1]).normalized();
    } else if (iPoint == &positions[1]) {
        return (positions[1] - positions[0]).normalized();
    } else {
        return Eigen::Vector3d(0, 0, 0);
    }
}

membrane_part_base::membrane_part_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        cellcomponents_base(iGlobals, iCell) {

}

std::set<linker_base *> &membrane_part_base::get_connectedLinkers() {
    return connectedLinkers;
}

/***************************
 * Matrix Compoents Base
 ***************************/

matrixcomponents_base::matrixcomponents_base(sGlobalVars &iGlobals) : components_base(iGlobals) {
    canColide = false;
    canMove = false;
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
