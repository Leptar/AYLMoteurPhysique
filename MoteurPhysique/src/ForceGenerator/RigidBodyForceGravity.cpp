#include "RigidBodyForceGravity.h"

RigidBodyForceGravity::RigidBodyForceGravity()
        : RigidBodyForceGravity(Vector3D(0.f, -9.81f, 0.f))
{
}

RigidBodyForceGravity::RigidBodyForceGravity(const Vector3D& gravity)
        : m_gravity(gravity)
        , m_enabled(true)
{
}

void RigidBodyForceGravity::setGravity(const Vector3D& gravity)
{
        m_gravity = gravity;
}

Vector3D RigidBodyForceGravity::getGravity() const
{
        return m_gravity;
}

void RigidBodyForceGravity::setEnabled(bool enabled)
{
        m_enabled = enabled;
}

bool RigidBodyForceGravity::isEnabled() const
{
        return m_enabled;
}

void RigidBodyForceGravity::UpdateForce(CorpsRigide& body, float dt)
{
        (void)dt;

        if (!m_enabled)
        {
                return;
        }

        float invMass = body.getInverseMasse();
        if (invMass <= 0.f)
        {
                return;
        }

        float mass = 1.f / invMass;
        body.addForce(m_gravity.scalar(mass));
}
