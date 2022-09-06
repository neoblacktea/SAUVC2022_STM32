#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "vector.h"
#include "quaternion.h"

struct Kinematics
{
    geometry::Vector linear;
    geometry::Vector angular;
};

struct Dynamics
{
    geometry::Vector position;
    Quaternion orientation;

    Kinematics velocity;
    Kinematics acceleration;
};

#endif