#pragma once

#include "CollisionPrimitive.h"
#include "RigidBodyContact.h"
#include <vector>

/**
 * @brief Classe responsable de la détection de collision en phase restreinte (narrow phase).
 * Implémente les algorithmes de détection pour différentes paires de primitives.
 */
class CollisionDetector
{
public:
    /**
     * @brief Détecte les collisions entre une boîte et un plan.
     * @param box La boîte à tester
     * @param plane Le plan à tester
     * @param contacts Tableau où stocker les contacts générés
     * @param maxContacts Nombre maximum de contacts à générer
     * @return Nombre de contacts générés
     */
    static unsigned boxAndPlane(const CollisionBox& box,
                                 const CollisionPlane& plane,
                                 RigidBodyContact* contacts,
                                 unsigned maxContacts);

    /**
     * @brief Détecte les collisions entre deux boîtes.
     * @param box1 Première boîte
     * @param box2 Seconde boîte
     * @param contacts Tableau où stocker les contacts générés
     * @param maxContacts Nombre maximum de contacts à générer
     * @return Nombre de contacts générés
     */
    static unsigned boxAndBox(const CollisionBox& box1,
                               const CollisionBox& box2,
                               RigidBodyContact* contacts,
                               unsigned maxContacts);

    /**
     * @brief Détecte les collisions entre deux sphères.
     * @param sphere1 Première sphère
     * @param sphere2 Seconde sphère
     * @param contacts Tableau où stocker le contact généré
     * @return 1 si collision, 0 sinon
     */
    static unsigned sphereAndSphere(const CollisionSphere& sphere1,
                                     const CollisionSphere& sphere2,
                                     RigidBodyContact* contacts);

    /**
     * @brief Détecte les collisions entre une sphère et un plan.
     * @param sphere La sphère à tester
     * @param plane Le plan à tester
     * @param contacts Tableau où stocker le contact généré
     * @return 1 si collision, 0 sinon
     */
    static unsigned sphereAndPlane(const CollisionSphere& sphere,
                                    const CollisionPlane& plane,
                                    RigidBodyContact* contacts);

private:
    /**
     * @brief Calcule la profondeur de pénétration d'un sommet dans un plan.
     */
    static float vertexPlaneDistance(const Vector3D& vertex, const CollisionPlane& plane);

    /**
     * @brief Teste si un point est à l'intérieur d'une boîte orientée.
     */
    static bool pointInBox(const Vector3D& point, const CollisionBox& box);
};
