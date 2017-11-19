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

void mf_linker::make_timeStep(const double &dT) {
    for (auto _it: connectedComponents) {
        if(auto _el = dynamic_cast<filament_base *>(_it)) {
            positions[0] = _el->get_positions()[1];
        }
    }
}
