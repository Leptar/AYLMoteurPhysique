#include "Octree.h"
#include <algorithm>

Octree::Octree(const Vector3D& center, float halfSize, int maxDepth, int maxObjects)
    : m_region(center, halfSize)
    , m_maxDepth(maxDepth)
    , m_currentDepth(0)
    , m_maxObjects(maxObjects)
{
    for (int i = 0; i < 8; i++) {
        m_children[i] = nullptr;
    }
}

Octree::~Octree()
{
    clear();
}

void Octree::clear()
{
    m_objects.clear();
    for (int i = 0; i < 8; i++) {
        m_children[i].reset();
    }
}

void Octree::subdivide()
{
    if (!isLeaf()) {
        return; // Déjà subdivisé
    }

    float childHalfSize = m_region.halfSize * 0.5f;
    float offset = childHalfSize;

    // Créer les 8 enfants
    for (int i = 0; i < 8; i++) {
        float offsetX = (i & 1) ? offset : -offset;
        float offsetY = (i & 2) ? offset : -offset;
        float offsetZ = (i & 4) ? offset : -offset;

        Vector3D childCenter = m_region.center + Vector3D(offsetX, offsetY, offsetZ);
        m_children[i] = std::make_unique<Octree>(childCenter, childHalfSize, m_maxDepth, m_maxObjects);
        m_children[i]->m_currentDepth = m_currentDepth + 1;
    }

    // Redistribuer les objets existants dans les enfants
    std::vector<CollisionPrimitive*> remainingObjects;

    for (CollisionPrimitive* obj : m_objects) {
        BoundingSphere sphere = obj->getBoundingSphere();
        int childIndex = getChildIndex(sphere);

        if (childIndex != -1) {
            m_children[childIndex]->insert(obj);
        } else {
            // L'objet chevauche plusieurs enfants, le garder à ce niveau
            remainingObjects.push_back(obj);
        }
    }

    m_objects = remainingObjects;
}

int Octree::getChildIndex(const BoundingSphere& sphere) const
{
    // Déterminer dans quel octant le centre de la sphère se trouve
    int index = 0;

    if (sphere.center.x > m_region.center.x) index |= 1;
    if (sphere.center.y > m_region.center.y) index |= 2;
    if (sphere.center.z > m_region.center.z) index |= 4;

    // Vérifier si la sphère est entièrement contenue dans cet enfant
    if (m_children[index] && m_children[index]->getRegion().contains(sphere)) {
        return index;
    }

    return -1; // La sphère chevauche plusieurs enfants
}

void Octree::insert(CollisionPrimitive* primitive)
{
    if (!primitive) {
        return;
    }

    BoundingSphere sphere = primitive->getBoundingSphere();

    // Si ce nœud n'est pas une feuille, essayer d'insérer dans un enfant
    if (!isLeaf()) {
        int childIndex = getChildIndex(sphere);
        if (childIndex != -1) {
            m_children[childIndex]->insert(primitive);
            return;
        }
        // Sinon, l'objet chevauche plusieurs enfants, le garder à ce niveau
        m_objects.push_back(primitive);
        return;
    }

    // C'est une feuille, ajouter l'objet
    m_objects.push_back(primitive);

    // Si on dépasse la capacité et qu'on peut subdiviser, le faire
    if (m_objects.size() > static_cast<size_t>(m_maxObjects) && m_currentDepth < m_maxDepth) {
        subdivide();
    }
}

void Octree::checkInternalCollisions(std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>>& pairs) const
{
    // Tester toutes les paires d'objets dans ce nœud
    for (size_t i = 0; i < m_objects.size(); i++) {
        for (size_t j = i + 1; j < m_objects.size(); j++) {
            BoundingSphere sphere1 = m_objects[i]->getBoundingSphere();
            BoundingSphere sphere2 = m_objects[j]->getBoundingSphere();

            if (sphere1.overlaps(sphere2)) {
                pairs.push_back(std::make_pair(m_objects[i], m_objects[j]));
            }
        }
    }
}

void Octree::collectPotentialCollisions(std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>>& pairs) const
{
    // Vérifier les collisions internes à ce nœud
    checkInternalCollisions(pairs);

    if (!isLeaf()) {
        // Récursivement collecter dans les enfants
        for (int i = 0; i < 8; i++) {
            if (m_children[i]) {
                m_children[i]->collectPotentialCollisions(pairs);
            }
        }

        // Vérifier les collisions entre objets de ce nœud et objets des enfants
        for (CollisionPrimitive* obj : m_objects) {
            BoundingSphere sphere = obj->getBoundingSphere();

            for (int i = 0; i < 8; i++) {
                if (m_children[i] && m_children[i]->getRegion().overlaps(sphere)) {
                    // Tester avec tous les objets de cet enfant
                    const std::vector<CollisionPrimitive*>& childObjects = m_children[i]->getObjects();
                    for (CollisionPrimitive* childObj : childObjects) {
                        BoundingSphere childSphere = childObj->getBoundingSphere();
                        if (sphere.overlaps(childSphere)) {
                            pairs.push_back(std::make_pair(obj, childObj));
                        }
                    }
                }
            }
        }
    }
}

void Octree::getPotentialCollisions(std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>>& pairs) const
{
    pairs.clear();
    collectPotentialCollisions(pairs);
}
