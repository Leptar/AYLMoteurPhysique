#include "CorpsRigide.h"
#include <cmath> 

// --- Constructeur ---
CorpsRigide::CorpsRigide() :
    m_position(0, 0, 0),
    m_velocite(0, 0, 0),
    m_inverseMasse(1.0f),
    m_orientation(1, 0, 0, 0),
    m_velociteAngulaire(0, 0, 0),
    m_linearDamping(0.99f),
    m_angularDamping(0.99f),
    m_forceAccum(0, 0, 0),
    m_torqueAccum(0, 0, 0)
{
    setInverseInertiaTensorBody(Matrix3::identity());
    
    _updateTransformMatrix();
    _updateInverseInertiaTensorWorld();
}

void CorpsRigide::setPosition(const Vector3D& pos)
{
    m_position = pos;
    _updateTransformMatrix();
}

void CorpsRigide::setVelocite(const Vector3D& vel)
{
    m_velocite = vel;
}

void CorpsRigide::setMasse(float masse)
{
    if (masse <= 0.0f) {
        m_inverseMasse = 0.0f;
    } else {
        m_inverseMasse = 1.0f / masse;
    }
}

void CorpsRigide::setOrientation(const Quaternion& q)
{
    m_orientation = q;
    
    _updateTransformMatrix();
    _updateInverseInertiaTensorWorld();
}

void CorpsRigide::setVelociteAngulaire(const Vector3D& velAng)
{
    m_velociteAngulaire = velAng;
}

void CorpsRigide::setInverseInertiaTensorBody(const Matrix3& tensor)
{
    m_inverseInertiaTensorBody = tensor;
    
    _updateInverseInertiaTensorWorld();
}

void CorpsRigide::setLinearDamping(float damping)
{
    m_linearDamping = damping;
}

void CorpsRigide::setAngularDamping(float damping)
{
    m_angularDamping = damping;
}

Vector3D CorpsRigide::getPosition()
{
    return m_position;
}

Quaternion CorpsRigide::getOrientation()
{
    return m_orientation;
}

const Matrix4& CorpsRigide::getTransformMatrix()
{
    return m_transformMatrix;
}

void CorpsRigide::addForce(const Vector3D& force)
{
    // À FAIRE
}

void CorpsRigide::addForceAtPoint(const Vector3D& force, const Vector3D& worldPoint)
{
    // À FAIRE
}

void CorpsRigide::addForceAtBodyPoint(const Vector3D& force, const Vector3D& bodyPoint)
{
    // À FAIRE
}

void CorpsRigide::clearAccumulators()
{
    // À FAIRE
}

void CorpsRigide::_updateTransformMatrix()
{
    // À FAIRE
}

void CorpsRigide::_updateInverseInertiaTensorWorld()
{
    // À FAIRE
}

void CorpsRigide::integrer(float deltaTime)
{
    // À FAIRE
}