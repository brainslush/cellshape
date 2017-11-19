#include "cBaseclasses.h"

#ifndef CELLFORMATION_CLINKER_H
#define CELLFORMATION_CLINKER_H

class mf_linker : public linker_base {
public:
    mf_linker(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~mf_linker();

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    void make_timeStep (const double &dT) override;
};

#endif //CELLFORMATION_CLINKER_H
