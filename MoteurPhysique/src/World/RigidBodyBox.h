#pragma once

#include "../CorpsRigide/CorpsRigide.h"
#include "ofColor.h"

struct RigidBodyBox
{
        CorpsRigide body;
        Vector3D halfExtents;
        float mass = 1.f;
        ofColor color = ofColor::white;
        float boundingRadius = 1.f;
        bool reachedGoal = false;
        bool outOfBounds = false;
};

