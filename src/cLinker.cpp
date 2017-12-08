#include "cLinker.h"

mf_linker::mf_linker(
        sGlobalVars &iGlobals,
        cell_base &iCell
) :
        linker_base(iGlobals, iCell) {
    associatedVisualObj = new visual_ellipse(this);
    associatedVisualObj->set_color(1,0,0);
    associatedVisualObj->set_fillColor(1,0,0);
    parameters.push_back(2);
    parameters.push_back(2);
    positions.push_back(Eigen::Vector3d(0,0,0));
}

mf_linker::~mf_linker() {
    delete associatedVisualObj;
    associatedVisualObj = nullptr;
}

void mf_linker::obtain_visualObjs(std::vector<visual_base *> &iVisualObjs) {
    iVisualObjs.push_back(associatedVisualObj);
}

void mf_linker::set_membranePositions(const std::pair<Vector3d *, Vector3d *> &iPos) {
    membranePositions = iPos;
}

std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &mf_linker::get_membranePositions() {
    return membranePositions;
}

void mf_linker::make_timeStep(const double &dT) {
    positions[0] = *referencePos;
    *membranePositions.first = *referencePos;
    *membranePositions.second = *referencePos;
}

Eigen::Vector3d *mf_linker::get_referencePos() {
    return referencePos;
}

void mf_linker::set_referencePos(Eigen::Vector3d *iReferencePos) {
    referencePos = iReferencePos;
}

std::pair<membrane_part_base *, membrane_part_base *> &mf_linker::get_connectedMembranes() {
    return connectedMembranes;
}

filament_base *mf_linker::get_connectedFillament() {
    return connectedFilament;
}

void mf_linker::set_connectedMembranes(const std::pair<membrane_part_base *, membrane_part_base *> &iMembranes) {
    connectedMembranes = iMembranes;
}

void mf_linker::set_connectedFillament(filament_base *iFilament) {
    connectedFilament = iFilament;
}

