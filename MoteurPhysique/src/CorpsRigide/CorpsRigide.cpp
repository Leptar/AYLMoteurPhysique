#include "CorpsRigide.h"
#include <cmath> 

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

/**
 * @brief Applique une force au centre de masse (pas de torque).
 */
void CorpsRigide::addForce(const Vector3D& force)
{
    m_forceAccum = m_forceAccum + force;
}

/**
 * @brief Applique une force à un point en coordonnées MONDE.
 * Cela génère une force ET un torque.
 */
void CorpsRigide::addForceAtPoint(const Vector3D& force, const Vector3D& worldPoint)
{
    m_forceAccum = m_forceAccum + force;
    Vector3D leverArm = worldPoint - m_position;
    Vector3D torque = leverArm.cross(force);
    m_torqueAccum = m_torqueAccum + torque;
}

/**
 * @brief Vide les accumulateurs de force et de torque.
 */
void CorpsRigide::clearAccumulators()
{
    m_forceAccum = Vector3D();
    m_torqueAccum = Vector3D();
}

/**
 * @brief Met à jour la matrice de transformation 4x4
 * en combinant l'orientation et la position actuelles.
 * Pour l'afficher dans le jeu de tir ballistique
 */
void CorpsRigide::_updateTransformMatrix()
{
    // cf. Chapitre 8 Conversion quaternion -> Matrices
    Matrix3 rotMatrix = m_orientation.toMatrix3();

    // cf. Chapitre 8 Transformations linéaires et affines
    m_transformMatrix.setRotation(rotMatrix);

    // cf. Chapitre 8 Transformations linéaires et affines
    m_transformMatrix.setPosition(m_position);

    // La dernière ligne doit bien être (0, 0, 0, 1).
    // A ajouter selon comment est implémenté Matrix4
}

/**
 * @brief Met à jour le tenseur d'inertie inverse 3x3
 * en le pivotant dans le système de coordonnées.
 * à utiliser avec le torque (m_torqueAccum) pour calculer l'accélération angulaire alpha
 */
void CorpsRigide::_updateInverseInertiaTensorWorld()
{
    /*
     * Formule : I_monde = R * I_local * R_transpose
     * Où R est la matrice de rotation 3x3 de notre orientation.
     * cf Chapitre 9 Notes d'implémentation
     */
    Matrix3 rotMatrix = m_orientation.toMatrix3();
    Matrix3 rotMatrixT = rotMatrix.transposed();
    m_inverseInertiaTensorWorld = rotMatrix * m_inverseInertiaTensorBody * rotMatrixT;
}

void CorpsRigide::integrer(float deltaTime)
{
    // À FAIRE
}