
#include "cActin.h"

actin::actin(
        sGlobalVars &iGlobals,
        cell_base &iCell,
        Eigen::Vector3d iStart,
        Eigen::Vector3d iTmVelocity,
        double iMaxLength,
        double iLifeTime,
        double iStallingForce
) :
        stokes::Base(),
        filament_base(iGlobals, iCell),
        tmVelocity(std::move(iTmVelocity)),
        birthTime(iGlobals.time),
        maxLength(iMaxLength),
        lifeTime(iLifeTime),
        stallingForce(iStallingForce) {
    associatedVisualObj->set_color(0, 0, 1, 1);
    associatedVisualObj->set_fillColor(0, 1, 0, 1);
    positions.clear();
    positions.push_back(iStart);
    positions.push_back(iStart);
    // test variables, remove them later
    double _length = (positions[0] - positions[1]).norm();
    /*
     * double _mass = _length;
    double _I = 0.83333333 * _mass * _length * _length;
    Eigen::Matrix3d _mI;
    _mI << 0, 0, 0,
            0, 0, 0,
            0, 0, _I;
    auto _mI2 = _mI.inverse();
    auto _mI3 = _mI2.diagonal();
    rigidBody = new physic::RigidBody3d(
            this,
            (positions[0] + positions[1]) / 2,
            physic::angleVector2d(tmVelocity),
            _mI,
            _mass,
            &iFunctors
    );
     */
    solver = new stokes::Solver (
        this,
        &iFunctors
    );
}

actin::~actin() = default;

Eigen::Vector3d &actin::get_tmVelocity() {
    return tmVelocity;
}

bool actin::make_timeStep(double &dT) {
    // destroy if it exceeds life time
    if (birthTime + lifeTime < globals.time) {
        return true;
    } else {
        auto length = (positions[0] - positions[1]).norm();
        positions[0] = positions[0] + globals.settings.deltaT * tmVelocity /2 ;
        positions[1] = positions[1] - globals.settings.deltaT * tmVelocity / 2;
        if (length > std::numeric_limits<double>::min()) {
            if (rigidBody) {
                // set new values like mass, MoI, position
                auto _length = (positions[1] - positions[0]).norm();
                auto _M = 1;
                double _I = 0.83333333 * _M * _length * _length;
                Eigen::Matrix3d _mI;
                _mI << 0, 0, 0,
                        0, 0, 0,
                        0, 0, _I;
                solver->set_mass(_M);
                rigidBody->set_inertia(_mI);
                rigidBody->set_position((positions[1] + positions[0]) / 2);
                auto _R2 = physic::angleVector2d(tmVelocity);
                rigidBody->set_rotation(_R2);
                // do one simulation step
                rigidBody->do_timeStep(dT);
                // apply simulated data onto visual model
                auto &_X = rigidBody->get_X();
                auto &_R = rigidBody->get_R();
                auto _l2 = 0.5 * (positions[1] - positions[0]).norm();
                auto _cos = cos(_R);
                auto _sin = sin(_R);
                Eigen::Vector3d _X2(_l2 * _cos, _l2 * _sin, 0);
                Eigen::Vector3d _X3 = _X - _X2;
                Eigen::Vector3d _X4 = _X + _X2;
                positions[1] = _X3;
                positions[0] = _X4;
                //tmVelocity = tmVelocity.norm() * Eigen::Vector3d(_cos,_sin,0);
            } else {
                std::cout << "actin w/o RigidBody\n";
            }
        }
        return false;
    }
}