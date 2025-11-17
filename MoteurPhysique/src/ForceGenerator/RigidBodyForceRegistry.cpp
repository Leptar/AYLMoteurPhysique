#include "RigidBodyForceRegistry.h"

#include <algorithm>

void RigidBodyForceRegistry::add(RigidBodyForceGenerator* generator)
{
        if (!generator)
        {
                return;
        }

        if (std::find(m_generators.begin(), m_generators.end(), generator) == m_generators.end())
        {
                m_generators.push_back(generator);
        }
}

void RigidBodyForceRegistry::remove(RigidBodyForceGenerator* generator)
{
        if (!generator)
        {
                return;
        }

        auto it = std::remove(m_generators.begin(), m_generators.end(), generator);
        m_generators.erase(it, m_generators.end());
}

void RigidBodyForceRegistry::clear()
{
        m_generators.clear();
}

void RigidBodyForceRegistry::updateForces(CorpsRigide& body, float dt) const
{
        for (RigidBodyForceGenerator* generator : m_generators)
        {
                if (generator)
                {
                        generator->UpdateForce(body, dt);
                }
        }
}
