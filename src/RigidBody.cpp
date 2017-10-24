#include "RigidBody.h"

using namespace physic;

Eigen::Quaterniond physic::vec2quat(const Vec3 &v) {
    Eigen::Quaterniond c;
    c.w() = 0;
    c.vec() = v;
    return c;
};

double physic::angleVector2d(
        const double &x1,
        const double &y1,
        const double &x2,
        const double &y2
) {
    auto _dot = x1 * x2 + y1 * y2;
    auto _det = x2 * y1 - x1 * y2;
    return atan2(_det, _dot);
}

double physic::angleVector2d(const Vec3 &v) {
    return angleVector2d(v(0), v(1), 1, 0);
}