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


/*
void components_base::make_timeStep(double &dT) {
    // do nothing
}
*/

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
        cell(iCell) {
}

cellcomponents_base::~cellcomponents_base() = default;

physic::RigidBody3d &cellcomponents_base::get_rigidBody() { return rigidBody; }

/***************************
 * crosslinker base
 ***************************/

crosslinker_base::crosslinker_base(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        cellcomponents_base(iGlobals, iCell) {
    canColide = false;
    canMove = true;
    force = Eigen::Vector3d(0, 0, 0);
    globals.grid->register_component(this);
};

crosslinker_base::~crosslinker_base() {
    globals.grid->unregister_component(this);
};

std::set<filament_base *> &crosslinker_base::get_connectedFilaments() {
    return connectedFilaments;
}

Eigen::Vector3d &crosslinker_base::get_force(unsigned long long iTimeStamp) {
    if (timeStamp != iTimeStamp) {
        return force;
    } else {
        return force;
    }
}

void crosslinker_base::add_connectedFilament(filament_base *iFilament) {
    connectedFilaments.insert(iFilament);
};

void crosslinker_base::remove_connectedFilament(filament_base *iFilament) {
    connectedFilaments.erase(iFilament);
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

void filament_base::add_connectedCrosslinker(crosslinker_base *iCrosslinker) {
    connectedCrosslinkers.insert(iCrosslinker);
}

void filament_base::remove_connectedCrosslinker(crosslinker_base *iCrosslinker) {
    connectedCrosslinkers.erase(iCrosslinker);
}

void filament_base::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

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
    restLength = get_restLength();
}

membrane_part_base::~membrane_part_base() = default;

std::pair<membrane_part_base *, membrane_part_base *> &membrane_part_base::get_neighbours() {
    return neighbours;
}

double membrane_part_base::get_length() {
    return (positions[1] - positions[0]).norm();
}

double &membrane_part_base::get_restLength() {
    return restLength;
}

std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &membrane_part_base::get_sharedPositions() {
    return sharedPositions;
}

void membrane_part_base::set_neighbours(std::pair<membrane_part_base *, membrane_part_base *> iNeighbours) {
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
