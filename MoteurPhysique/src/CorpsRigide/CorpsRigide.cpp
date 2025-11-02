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

void CorpsRigide::integrer(float deltaTime)
{
    // À FAIRE
}

/**
 * @brief Met à jour la matrice de transformation 4x4 (pour le rendu)
 * en combinant l'orientation et la position actuelles.
 */
void CorpsRigide::_updateTransformMatrix()
{
    // D'abord, on convertit le quaternion en sa matrice de rotation 3x3 
    // cf. Chapitre 8 Conversion quaternion -> Matrices
    Matrix3 rotMatrix = m_orientation.toMatrix3();

    // Ensuite, on dit à notre matrice 4x4 d'utiliser cette matrice 3x3
    // pour sa partie "rotation".
    // cf. Chapitre 8 Transformations linéaires et affines
    m_transformMatrix.setRotation(rotMatrix);
    
    // On dit à notre matrice 4x4 d'utiliser ce vecteur
    // pour sa partie "translation" (la 4ème colonne).
    // cf. Chapitre 8 Transformations linéaires et affines
    m_transformMatrix.setPosition(m_position);

    // Pour que cette matrice soit une matrice de transformation affine valide,
    // ces méthodes doivent garantir que la dernière ligne est bien (0, 0, 0, 1).
}

/**
 * @brief Met à jour le tenseur d'inertie inverse 3x3 (pour la physique)
 * en le pivotant dans le système de coordonnées du monde.
 */
void CorpsRigide::_updateInverseInertiaTensorWorld()
{
    /*
     * Formule : I_monde = R * I_local * R_transpose
     * Où R est la matrice de rotation 3x3 de notre orientation.
     */

    // 1. Obtenir la matrice de rotation 3x3 à partir du quaternion
    // (Cette fonction doit implémenter la formule que vous m'avez montrée)
    Matrix3 rotMatrix = m_orientation.toMatrix3();

    // 2. Obtenir la transposée de cette matrice
    // (suppose que votre classe Matrix3 a une méthode .transposed())
    Matrix3 rotMatrixT = rotMatrix.transposed();

    // 3. Calculer le tenseur monde par la triple multiplication
    // (suppose que l'opérateur * est surchargé pour Matrix3 * Matrix3)
    m_inverseInertiaTensorWorld = rotMatrix * m_inverseInertiaTensorBody * rotMatrixT;
}