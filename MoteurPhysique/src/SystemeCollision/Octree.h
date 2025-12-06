#pragma once

#include "../MathStruct/3DVector.h"
#include "BoundingSphere.h"
#include "CollisionPrimitive.h"
#include <vector>
#include <memory>

/**
 * @brief Représente une région cubique de l'espace dans l'octree.
 */
struct OctreeRegion
{
    Vector3D center;
    float halfSize;

    OctreeRegion() : center(), halfSize(0) {}
    OctreeRegion(const Vector3D& c, float hs) : center(c), halfSize(hs) {}

    /**
     * @brief Teste si cette région contient une sphère englobante.
     */
    bool contains(const BoundingSphere& sphere) const
    {
        // Vérifier si tous les points extrêmes de la sphère sont dans la région
        Vector3D min = center - Vector3D(halfSize, halfSize, halfSize);
        Vector3D max = center + Vector3D(halfSize, halfSize, halfSize);

        return (sphere.center.x - sphere.radius >= min.x &&
                sphere.center.y - sphere.radius >= min.y &&
                sphere.center.z - sphere.radius >= min.z &&
                sphere.center.x + sphere.radius <= max.x &&
                sphere.center.y + sphere.radius <= max.y &&
                sphere.center.z + sphere.radius <= max.z);
    }

    /**
     * @brief Teste si cette région chevauche une sphère englobante.
     */
    bool overlaps(const BoundingSphere& sphere) const
    {
        // Trouver le point le plus proche dans la boîte
        Vector3D min = center - Vector3D(halfSize, halfSize, halfSize);
        Vector3D max = center + Vector3D(halfSize, halfSize, halfSize);

        float closestX = std::max(min.x, std::min(sphere.center.x, max.x));
        float closestY = std::max(min.y, std::min(sphere.center.y, max.y));
        float closestZ = std::max(min.z, std::min(sphere.center.z, max.z));

        Vector3D closest(closestX, closestY, closestZ);
        float distSquared = (closest - sphere.center).GetNormSquared();

        return distSquared < (sphere.radius * sphere.radius);
    }
};

/**
 * @brief Structure de données Octree pour la phase élargie de détection de collision.
 * L'octree partitionne l'espace en 8 sous-régions récursivement.
 */
class Octree
{
public:
    /**
     * @brief Construit un octree couvrant la région spécifiée.
     * @param center Centre de la région racine
     * @param halfSize Demi-taille de la région racine
     * @param maxDepth Profondeur maximale de l'arbre
     * @param maxObjects Nombre maximum d'objets par nœud avant subdivision
     */
    Octree(const Vector3D& center, float halfSize, int maxDepth = 6, int maxObjects = 8);
    ~Octree();

    /**
     * @brief Insère une primitive de collision dans l'octree.
     */
    void insert(CollisionPrimitive* primitive);

    /**
     * @brief Efface tous les objets de l'octree.
     */
    void clear();

    /**
     * @brief Retourne toutes les paires potentielles de collision.
     */
    void getPotentialCollisions(std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>>& pairs) const;

    /**
     * @brief Retourne la région couverte par ce nœud.
     */
    const OctreeRegion& getRegion() const { return m_region; }

    /**
     * @brief Retourne vrai si ce nœud est une feuille.
     */
    bool isLeaf() const { return m_children[0] == nullptr; }

    /**
     * @brief Retourne les objets contenus dans ce nœud (seulement pour les feuilles).
     */
    const std::vector<CollisionPrimitive*>& getObjects() const { return m_objects; }

    /**
     * @brief Retourne les enfants de ce nœud (nullptr si c'est une feuille).
     */
    Octree* getChild(int index) const { return m_children[index].get(); }

private:
    OctreeRegion m_region;
    int m_maxDepth;
    int m_currentDepth;
    int m_maxObjects;

    std::vector<CollisionPrimitive*> m_objects;
    std::unique_ptr<Octree> m_children[8];

    /**
     * @brief Subdivise ce nœud en 8 enfants.
     */
    void subdivide();

    /**
     * @brief Détermine dans quel enfant (0-7) un objet devrait aller.
     * Retourne -1 si l'objet ne rentre dans aucun enfant unique.
     */
    int getChildIndex(const BoundingSphere& sphere) const;

    /**
     * @brief Collecte les collisions potentielles dans ce sous-arbre.
     */
    void collectPotentialCollisions(std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>>& pairs) const;

    /**
     * @brief Teste toutes les paires d'objets dans ce nœud.
     */
    void checkInternalCollisions(std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>>& pairs) const;
};
