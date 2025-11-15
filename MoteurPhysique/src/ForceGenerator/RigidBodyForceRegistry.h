#pragma once

#include <vector>

#include "RigidBodyForceGenerator.h"

class RigidBodyForceRegistry
{
public:
        void add(RigidBodyForceGenerator* generator);
        void remove(RigidBodyForceGenerator* generator);
        void clear();

        void updateForces(CorpsRigide& body, float dt) const;

private:
        std::vector<RigidBodyForceGenerator*> m_generators;
};
