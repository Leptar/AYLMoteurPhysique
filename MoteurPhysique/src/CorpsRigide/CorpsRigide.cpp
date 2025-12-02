#include "CorpsRigide.h"
#include <cmath>
#include <algorithm> // Pour std::min et std::max
#include "WorldObject/Primitive.h"
#include "WorldObject/Box.h"
#include "WorldObject/Sphere.h"

CorpsRigide::CorpsRigide() :
    m_position(0.f, 0.f, 0.f),
    m_velocite(0.f, 0.f, 0.f),
    m_inverseMasse(1.0f),
    m_orientation(1.f, 0.f, 0.f, 0.f),
    m_velociteAngulaire(0.f, 0.f, 0.f),
    m_inverseInertiaTensorBody(Matrix3::Identity()),
    m_inverseInertiaTensorWorld(Matrix3::Identity()),
    m_linearDamping(0.99f),
    m_angularDamping(0.99f),
    m_forceAccum(0.f, 0.f, 0.f),
    m_torqueAccum(0.f, 0.f, 0.f),
    m_transformMatrix(),
    worldAABB()
{
    _updateTransformMatrix();
    _updateInverseInertiaTensorWorld();
}

// --- Paramétrage masse / inertie ---

void CorpsRigide::setInverseMasse(float invMasse)
{
    m_inverseMasse = invMasse;
}

float CorpsRigide::getInverseMasse() const
{
    return m_inverseMasse;
}

void CorpsRigide::setInverseInertiaTensorBody(const Matrix3& invIbody)
{
    m_inverseInertiaTensorBody = invIbody;
    _updateInverseInertiaTensorWorld();
}

const Matrix3& CorpsRigide::getInverseInertiaTensorWorld() const
{
    return m_inverseInertiaTensorWorld;
}

// --- État linéaire ---

void CorpsRigide::setPosition(const Vector3D& p)
{
    m_position = p;
    _updateTransformMatrix();
}

void CorpsRigide::setVelocite(const Vector3D& v)
{
    m_velocite = v;
}

Vector3D CorpsRigide::getPosition() const
{
    return m_position;
}

Vector3D CorpsRigide::getVelocite() const
{
    return m_velocite;
}

// --- État angulaire ---

void CorpsRigide::setOrientation(const Quaternion& q)
{
    m_orientation = q;
    m_orientation.normalize();
    _updateTransformMatrix();
    _updateInverseInertiaTensorWorld();
}

void CorpsRigide::setVelociteAngulaire(const Vector3D& w)
{
    m_velociteAngulaire = w;
}

Quaternion CorpsRigide::getOrientation() const
{
    return m_orientation;
}

Vector3D CorpsRigide::getVelociteAngulaire() const
{
    return m_velociteAngulaire;
}

// --- Transform ---

const Matrix4& CorpsRigide::getTransformMatrix() const
{
    return m_transformMatrix;
}

// --- Forces / torques ---

void CorpsRigide::addForce(const Vector3D& force)
{
    m_forceAccum = m_forceAccum + force;
}

void CorpsRigide::addForceAtPoint(const Vector3D& force, const Vector3D& pointWorld)
{
    // pointWorld est en coordonnées monde
    // r = point - centre
    Vector3D r = pointWorld - m_position;

    // Force au centre de masse
    addForce(force);

    // Torque = r x F
    Vector3D torque = r.cross(force);
    m_torqueAccum = m_torqueAccum + torque;
}

void CorpsRigide::clearAccumulators()
{
    m_forceAccum  = Vector3D(0.f, 0.f, 0.f);
    m_torqueAccum = Vector3D(0.f, 0.f, 0.f);
}

// --- Collision ---

void CorpsRigide::calculateWorldAABB(const Primitive& primitive)
{
    AABB localAABB;

    // On utilise dynamic_cast pour déterminer le type de primitive et obtenir sa AABB locale.
    if (const Box* box = dynamic_cast<const Box*>(&primitive))
    {
        localAABB.min = box->HalfExtent.scalar(-1.f);
        localAABB.max = box->HalfExtent;
    }
    else if (const Sphere* sphere = dynamic_cast<const Sphere*>(&primitive))
    {
        localAABB.min = Vector3D(-sphere->radius, -sphere->radius, -sphere->radius);
        localAABB.max = Vector3D( sphere->radius,  sphere->radius,  sphere->radius);
    }
    else
    {
        // Type de primitive non géré, on laisse une AABB nulle.
        worldAABB = AABB();
        return;
    }

    // On récupère les 8 coins de la AABB locale
    Vector3D corners[8];
    corners[0] = Vector3D(localAABB.min.x, localAABB.min.y, localAABB.min.z);
    corners[1] = Vector3D(localAABB.max.x, localAABB.min.y, localAABB.min.z);
    corners[2] = Vector3D(localAABB.min.x, localAABB.max.y, localAABB.min.z);
    corners[3] = Vector3D(localAABB.max.x, localAABB.max.y, localAABB.min.z);
    corners[4] = Vector3D(localAABB.min.x, localAABB.min.y, localAABB.max.z);
    corners[5] = Vector3D(localAABB.max.x, localAABB.min.y, localAABB.max.z);
    corners[6] = Vector3D(localAABB.min.x, localAABB.max.y, localAABB.max.z);
    corners[7] = Vector3D(localAABB.max.x, localAABB.max.y, localAABB.max.z);

    // On transforme chaque coin dans l'espace monde
    for (int i = 0; i < 8; ++i)
    {
        corners[i] = m_transformMatrix * corners[i];
    }

    // On trouve les nouvelles bornes min/max pour créer la AABB monde
    Vector3D min = corners[0];
    Vector3D max = corners[0];

    for (int i = 1; i < 8; ++i)
    {
        min.x = std::min(min.x, corners[i].x);
        min.y = std::min(min.y, corners[i].y);
        min.z = std::min(min.z, corners[i].z);

        max.x = std::max(max.x, corners[i].x);
        max.y = std::max(max.y, corners[i].y);
        max.z = std::max(max.z, corners[i].z);
    }
    worldAABB = AABB(min, max);
}

// --- Helpers internes ---

void CorpsRigide::_updateTransformMatrix()
{
    Matrix3 R = m_orientation.toMatrix3();

    Matrix4 M; // identité par défaut
    M.SetRotation(R);

    Vector3D pos = m_position;
    M.SetPosition(pos);

    m_transformMatrix = M;
}

void CorpsRigide::_updateInverseInertiaTensorWorld()
{
    Matrix3 R  = m_orientation.toMatrix3();
    Matrix3 Rt = R.transpose();
    m_inverseInertiaTensorWorld = R * m_inverseInertiaTensorBody * Rt;
}

// --- Intégration ---

void CorpsRigide::integrer(float deltaTime)
{
    if (deltaTime <= 0.f || m_inverseMasse <= 0.f)
    {
        clearAccumulators();
        return;
    }

    // 1) Accélération linéaire : a = F * invM
    Vector3D acceleration = m_forceAccum.scalar(m_inverseMasse);

    // v_{t+dt} = v_t + a * dt
    m_velocite = m_velocite + acceleration.scalar(deltaTime);

    // Damping exponentiel
    float linDamp = std::pow(m_linearDamping, deltaTime);
    m_velocite = m_velocite.scalar(linDamp);

    // x_{t+dt} = x_t + v_{t+dt} * dt
    m_position = m_position + m_velocite.scalar(deltaTime);

    // 2) Accélération angulaire : alpha = I_world^{-1} * torque
    Vector3D angAcc = m_inverseInertiaTensorWorld * m_torqueAccum;

    // omega_{t+dt} = omega_t + alpha * dt
    m_velociteAngulaire = m_velociteAngulaire + angAcc.scalar(deltaTime);
    float angDamp = std::pow(m_angularDamping, deltaTime);
    m_velociteAngulaire = m_velociteAngulaire.scalar(angDamp);

    // 3) Mise à jour de l’orientation
    // dq/dt = 0.5 * w_quat * q, avec w_quat = (0, omega)
    Quaternion w_quat(
        0.f,
        m_velociteAngulaire.x,
        m_velociteAngulaire.y,
        m_velociteAngulaire.z
    );

    Quaternion q_dot = w_quat * m_orientation;
    float halfDt = 0.5f * deltaTime;

    Quaternion q_delta(
        q_dot.w * halfDt,
        q_dot.x * halfDt,
        q_dot.y * halfDt,
        q_dot.z * halfDt
    );

    m_orientation = m_orientation + q_delta;
    m_orientation.normalize();

    // 4) Mise à jour des matrices
    _updateTransformMatrix();
    _updateInverseInertiaTensorWorld();

    // 5) Nettoyage des forces / torques
    clearAccumulators();
}
