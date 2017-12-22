
#include "cActin.h"

actin::actin(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        const Eigen::Vector3d &iStart,
        const Eigen::Vector3d &iTmVelocity,
        const double &iMaxLength,
        const double &iLifeTime,
        const double &iStallingForce,
        const double &iStokesCoeff,
        std::set<stokes::functor *> &iFunctors
) :
        filament_base(iGlobals, iCell),
        tmVelocity(std::move(iTmVelocity)),
        birthTime(iGlobals.time),
        maxLength(iMaxLength),
        lifeTime(iLifeTime),
        stallingForce(iStallingForce),
        stokesCoeff(iStokesCoeff)
{
    associatedVisualObj->set_color(0, 0, 1, 1);
    associatedVisualObj->set_fillColor(0, 1, 0, 1);
    positions.clear();
    positions.push_back(iStart);
    positions.push_back(iStart);
    // create solver
    solver = new stokes::Solver (
            this,
            &iFunctors
    );
    solver->set_c(stokesCoeff);
}

actin::~actin() {
    cell.unregister_filament(this);
}

Eigen::Vector3d &actin::get_tmVelocity() {
    return tmVelocity;
}

bool actin::make_timeStep(double &dT) {
    // destroy if it exceeds life time
    if (birthTime + lifeTime < globals.time) {
        positions[0] = positions[0] - globals.settings.deltaT * tmVelocity /2 ;
        positions[1] = positions[1] + globals.settings.deltaT * tmVelocity / 2;
        auto _l = (positions[1] - positions[0]).norm();
        if (_l <= tmVelocity.norm()) {
            return true;
        }
    } else {
        auto _l = (positions[1] - positions[0]).norm();
        if (_l < maxLength) {
            positions[0] = positions[0] + globals.settings.deltaT * tmVelocity /2 ;
            positions[1] = positions[1] - globals.settings.deltaT * tmVelocity / 2;
        }
    }
    auto _l = (positions[1] - positions[0]).norm();
    if (_l > std::numeric_limits<double>::min()) {
        if (solver) {
            // set new values like mass, MoI, position
            solver->set_X((positions[1] + positions[0]) / 2);
            Eigen::Vector3d _dirV = (positions[1] - positions[0]).normalized();
            auto _R2 = bmath::angleVector2d(_dirV(0),_dirV(1));
            solver->set_R(_R2);
            // do one simulation step
            solver->do_timeStep(dT);
            // apply simulated data onto visual model
            auto &_X = solver->get_X();
            auto &_R = solver->get_R();
            auto _l2 = 0.5 * (positions[1] - positions[0]).norm();
            auto _cos = cos(_R);
            auto _sin = sin(_R);
            Eigen::Vector3d _X2(_l2 * _cos, _l2 * _sin, 0);
            positions[0] = _X - _X2;
            positions[1] = _X + _X2;
        } else {
            std::cout << "actin w/o RigidBody\n";
        }
    }
    return false;
}