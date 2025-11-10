#pragma once

#include "MathStruct/3DVector.h"
#include "MathStruct/Quaternion.h"
#include "MathStruct/Matrix3.h"
#include "MathStruct/Matrix4.h"

class CorpsRigide
{
private:
    // Etat
    Vector3D m_position;
    Vector3D m_velocite;
    float    m_inverseMasse;

    Quaternion m_orientation;
    Vector3D   m_velociteAngulaire;

    // Propriétés physiques
    Matrix3  m_inverseInertiaTensorBody;
    float m_linearDamping;
    float m_angularDamping;

    // Forces
    Vector3D m_forceAccum;
    Vector3D m_torqueAccum;

    // Jeu de tir ballistique
    Matrix4 m_transformMatrix;
    Matrix3 m_inverseInertiaTensorWorld;

    // Pour chaque frame
    void _updateTransformMatrix();
    void _updateInverseInertiaTensorWorld();

public:
    CorpsRigide();
    // Setters
    void setPosition(const Vector3D& pos);
    void setVelocite(const Vector3D& vel);
    void setMasse(float masse);
    void setOrientation(const Quaternion& q);
    void setVelociteAngulaire(const Vector3D& velAng);

    void setInverseInertiaTensorBody(const Matrix3& tensor);
    void setLinearDamping(float damping);
    void setAngularDamping(float damping);

    // Getters
    Vector3D getPosition();
    Quaternion getOrientation();
    const Matrix4& getTransformMatrix();

    // Gestion des forces et torques
    void addForce(const Vector3D& force); // Centre de masse
    void addForceAtPoint(const Vector3D& force, const Vector3D& worldPoint); // Coordonnées dans le monde (Force externe appliquée n'importe où sur le corps)
    void clearAccumulators();

    // Integrer
    void integrer(float deltaTime);
};
