#pragma once

#include "../CorpsRigide/CorpsRigide.h"

class RigidBodyForceGenerator
{
public:
        virtual ~RigidBodyForceGenerator() = default;
        virtual void UpdateForce(CorpsRigide& body, float dt) = 0;
};
