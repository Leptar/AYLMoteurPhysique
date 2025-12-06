#pragma once

#include "../MathStruct/3DVector.h"
#include "../CorpsRigide/CorpsRigide.h"

/**
 * @brief Représente un contact entre deux corps rigides ou un corps rigide et un plan.
 * Contient toutes les informations nécessaires pour résoudre le contact.
 */
struct RigidBodyContact
{
    // Les deux corps en contact (body[1] peut être nullptr pour un contact avec un plan/objet immobile)
    CorpsRigide* body[2];

    // Coefficient de restitution (0 = inélastique, 1 = parfaitement élastique)
    float restitution;

    // Normale du contact (pointe de body[0] vers body[1])
    Vector3D contactNormal;

    // Profondeur de pénétration
    float penetration;

    // Point de contact dans l'espace monde
    Vector3D contactPoint;

    // Coefficient de friction
    float friction;

    /**
     * @brief Résout ce contact en appliquant une impulsion aux corps.
     * @param deltaTime Le pas de temps de la simulation
     */
    void resolve(float deltaTime);

protected:
    /**
     * @brief Calcule la vitesse de séparation au point de contact.
     * @return Vitesse de séparation (négative si les corps s'approchent)
     */
    float calculateSeparatingVelocity() const;

    /**
     * @brief Résout la vitesse du contact (applique l'impulsion).
     * @param deltaTime Le pas de temps de la simulation
     */
    void resolveVelocity(float deltaTime);

    /**
     * @brief Résout l'interpénétration en déplaçant les corps.
     * @param deltaTime Le pas de temps de la simulation
     */
    void resolveInterpenetration(float deltaTime);
};

/**
 * @brief Gère une collection de contacts et les résout de manière itérative.
 */
class ContactResolver
{
public:
    ContactResolver(unsigned iterations);

    /**
     * @brief Résout tous les contacts fournis.
     * @param contacts Tableau de contacts à résoudre
     * @param numContacts Nombre de contacts
     * @param deltaTime Pas de temps de la simulation
     */
    void resolveContacts(RigidBodyContact* contacts, unsigned numContacts, float deltaTime);

    void setIterations(unsigned iterations);

protected:
    unsigned m_iterations;
    unsigned m_iterationsUsed;
};
