//
// Created by brainslush on 02/12/17.
//

#include "cLinkerFunctor.h"


functor_cell_linkerCreation::functor_cell_linkerCreation(sGlobalVars &iGlobals) :
        functor_linker_base(iGlobals, "", "") {

}

functor_cell_linkerCreation::~functor_cell_linkerCreation() {

}

linker_base *functor_cell_linkerCreation::create_linker(
        cell_base *iCell,
        const std::set<cellcomponents_base *> &iConnectedComponents
) {
    /*auto _newLinker = new mf_linker(globals, *iCell);
    _newLinker->set_connectedComponents(iConnectedComponents);
    //_newLinker->set_membranePositions({nullptr, nullptr});
    iCell->register_linker(_newLinker);
    return _newLinker;*/
    return nullptr;
}

void functor_cell_linkerCreation::delete_linker(cell_base *iCell, linker_base *iLinker) {
    /*auto &_components = iLinker->get_connectedComponents();
    iCell->unregister_linker(iLinker);
    for (auto _it : _components) {
        _it->remove_connectedLinker(iLinker);
    }
    delete iLinker;
    iLinker = nullptr;
     */
}

