#pragma once

#include "RigidBodyForceGenerator.h"

class RigidBodyForceGravity : public RigidBodyForceGenerator
{
public:
        RigidBodyForceGravity();
        explicit RigidBodyForceGravity(const Vector3D& gravity);

        void setGravity(const Vector3D& gravity);
        Vector3D getGravity() const;

        void setEnabled(bool enabled);
        bool isEnabled() const;

        void UpdateForce(CorpsRigide& body, float dt) override;

private:
        Vector3D m_gravity;
        bool m_enabled;
};
