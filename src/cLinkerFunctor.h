//
// Created by brainslush on 02/12/17.
//

#pragma once

#include "cBaseclasses.h"
#include "cCell.h"
#include "cLinker.h"

#ifndef CELLFORMATION_CLINKERFUNCTOR_H
#define CELLFORMATION_CLINKERFUNCTOR_H

class cell;

class functor_cell_linkerCreation : public functor_linker_base {
public:
    functor_cell_linkerCreation(sGlobalVars &iGlobals);

    virtual ~functor_cell_linkerCreation();

    virtual linker_base *create_linker(
            cell_base *iCell,
            const std::set<cellcomponents_base *> &iConnectedComponents
    );

    virtual void delete_linker(cell_base *iCell, linker_base *iLinker);

protected:

};


#endif //CELLFORMATION_CLINKERFUNCTOR_H
