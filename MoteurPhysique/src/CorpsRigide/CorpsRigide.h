#pragma once

#include "MathStruct/3DVector.h"
#include "MathStruct/Quaternion.h"
#include "MathStruct/Matrix3.h"
#include "MathStruct/Matrix4.h"
#include "MathStruct/AABB.h"

class Primitive; // Déclaration avancée pour éviter une dépendance circulaire

class CorpsRigide
{
public:
    CorpsRigide();

    // Paramétrage masse / inertie
    void setInverseMasse(float invMasse);                 // 1/m (0 => immobile)
    float getInverseMasse() const;

    void setInverseInertiaTensorBody(const Matrix3& invIbody);
    const Matrix3& getInverseInertiaTensorWorld() const;

    // État linéaire
    void setPosition(const Vector3D& p);
    void setVelocite(const Vector3D& v);

    Vector3D getPosition() const;
    Vector3D getVelocite() const;

    // État angulaire
    void setOrientation(const Quaternion& q);
    void setVelociteAngulaire(const Vector3D& w);

    Quaternion getOrientation() const;
    Vector3D   getVelociteAngulaire() const;

    // Matrice de transform (pour l'affichage)
    const Matrix4& getTransformMatrix() const;

    // Gestion des forces et torques
    void addForce(const Vector3D& force);                 // au centre de masse
    void addForceAtPoint(const Vector3D& force,
                         const Vector3D& pointWorld);     // point en coordonnées monde
    void clearAccumulators();

    // Intégration (une étape de simulation)
    void integrer(float deltaTime);

    // --- Collision ---
    // Calcule et met à jour la AABB du corps rigide en coordonnées monde
    void calculateWorldAABB(const Primitive& primitive);

    AABB worldAABB;

private:
    // État linéaire
    Vector3D m_position;
    Vector3D m_velocite;
    float    m_inverseMasse;   // 1 / masse

    // État angulaire
    Quaternion m_orientation;
    Vector3D   m_velociteAngulaire; // omega

    // Inertie
    Matrix3 m_inverseInertiaTensorBody;   // dans l'espace corps
    Matrix3 m_inverseInertiaTensorWorld;  // dans l'espace monde

    // Amortissement
    float m_linearDamping;
    float m_angularDamping;

    // Accumulateurs de forces / torques
    Vector3D m_forceAccum;
    Vector3D m_torqueAccum;

    // Matrice de transform pour le rendu
    Matrix4 m_transformMatrix;

    // Helpers internes
    void _updateTransformMatrix();
    void _updateInverseInertiaTensorWorld();
};
