#pragma once

#include "Octree.h"
#include "CollisionPrimitive.h"
#include "CollisionDetector.h"
#include "RigidBodyContact.h"
#include <vector>
#include <memory>

/**
 * @brief Gestionnaire principal du système de détection et résolution de collisions.
 * Coordonne la phase élargie (octree) et la phase restreinte (détection précise).
 */
class CollisionWorld
{
public:
    /**
     * @brief Construit un monde de collision.
     * @param worldSize Taille du monde (utilisée pour l'octree)
     * @param maxContacts Nombre maximum de contacts pouvant être gérés
     */
    CollisionWorld(float worldSize = 1000.0f, unsigned maxContacts = 256);
    ~CollisionWorld();

    /**
     * @brief Enregistre une primitive de collision dans le système.
     */
    void addPrimitive(CollisionPrimitive* primitive);

    /**
     * @brief Enregistre un plan de collision (typiquement pour les limites du monde).
     */
    void addPlane(const CollisionPlane& plane);

    /**
     * @brief Efface toutes les primitives enregistrées.
     */
    void clearPrimitives();

    /**
     * @brief Effectue la détection de collisions et génère les contacts.
     * @return Nombre de contacts générés
     */
    unsigned detectCollisions();

    /**
     * @brief Résout tous les contacts détectés.
     * @param deltaTime Pas de temps de la simulation
     */
    void resolveCollisions(float deltaTime);

    /**
     * @brief Retourne le nombre de contacts actuellement détectés.
     */
    unsigned getContactCount() const { return m_contactCount; }

    /**
     * @brief Retourne les contacts détectés.
     */
    const RigidBodyContact* getContacts() const { return m_contacts; }

    /**
     * @brief Retourne l'octree pour visualisation.
     */
    const Octree* getOctree() const { return m_octree.get(); }

    /**
     * @brief Active ou désactive la visualisation de l'octree.
     */
    void setOctreeVisualizationEnabled(bool enabled) { m_visualizeOctree = enabled; }
    bool isOctreeVisualizationEnabled() const { return m_visualizeOctree; }

    /**
     * @brief Reconstruit l'octree (à appeler lorsque les objets bougent).
     */
    void rebuildOctree();

private:
    std::unique_ptr<Octree> m_octree;
    std::vector<CollisionPrimitive*> m_primitives;
    std::vector<CollisionPlane> m_planes;

    RigidBodyContact* m_contacts;
    unsigned m_maxContacts;
    unsigned m_contactCount;

    std::unique_ptr<ContactResolver> m_resolver;
    bool m_visualizeOctree;

    /**
     * @brief Génère les contacts pour une paire de primitives.
     */
    unsigned generateContacts(CollisionPrimitive* prim1, CollisionPrimitive* prim2);

    /**
     * @brief Génère les contacts entre une primitive et tous les plans.
     */
    unsigned generatePlaneContacts(CollisionPrimitive* prim);
};
