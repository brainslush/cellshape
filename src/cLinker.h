#include "cBaseclasses.h"

#ifndef CELLFORMATION_CLINKER_H
#define CELLFORMATION_CLINKER_H

class mf_linker : public linker_base {
public:
    mf_linker(sGlobalVars &iGlobals, cell_base &iCell);

    virtual ~mf_linker();

    virtual std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &get_membranePositions();

    virtual Eigen::Vector3d *get_referencePos();

    virtual std::pair<membrane_part_base *, membrane_part_base *> &get_connectedMembranes();

    virtual filament_base *get_connectedFillament();

    virtual void set_membranePositions(const std::pair<Eigen::Vector3d *, Eigen::Vector3d *> &iPos);

    virtual void set_connectedMembranes(const std::pair<membrane_part_base *, membrane_part_base *> &iMembranes);

    virtual void set_connectedFillament(filament_base *iFilament);

    virtual void set_referencePos(Eigen::Vector3d *iReferencePos);

    virtual void obtain_visualObjs(std::vector<visual_base *> &iVisualObjs);

    void make_timeStep (const double &dT) override;

protected:
    std::pair<membrane_part_base *, membrane_part_base *> connectedMembranes;
    filament_base *connectedFilament;
    std::pair<Eigen::Vector3d *, Eigen::Vector3d *> membranePositions;
    Eigen::Vector3d *referencePos;
};

#endif //CELLFORMATION_CLINKER_H
