#include "RigidBody.h"

using namespace physic;

Eigen::Quaterniond physic::vec2quat(const Vec3 &v) {
    Eigen::Quaterniond c;
    c.w() = 0;
    c.vec() = v;
    return c;
};