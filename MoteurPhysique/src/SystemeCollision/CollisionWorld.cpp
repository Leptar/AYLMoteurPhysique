#include "CollisionWorld.h"

CollisionWorld::CollisionWorld(float worldSize, unsigned maxContacts)
    : m_maxContacts(maxContacts)
    , m_contactCount(0)
    , m_visualizeOctree(false)
{
    // Créer l'octree centré à l'origine
    m_octree = std::make_unique<Octree>(Vector3D(0, 0, 0), worldSize / 2.0f);

    // Allouer le tableau de contacts
    m_contacts = new RigidBodyContact[maxContacts];

    // Créer le résolveur de contacts
    m_resolver = std::make_unique<ContactResolver>(maxContacts * 8);
}

CollisionWorld::~CollisionWorld()
{
    delete[] m_contacts;
}

void CollisionWorld::addPrimitive(CollisionPrimitive* primitive)
{
    if (primitive) {
        m_primitives.push_back(primitive);
    }
}

void CollisionWorld::addPlane(const CollisionPlane& plane)
{
    m_planes.push_back(plane);
}

void CollisionWorld::clearPrimitives()
{
    m_primitives.clear();
    m_planes.clear();
    m_octree->clear();
}

void CollisionWorld::rebuildOctree()
{
    m_octree->clear();

    for (CollisionPrimitive* prim : m_primitives) {
        m_octree->insert(prim);
    }
}

unsigned CollisionWorld::generateContacts(CollisionPrimitive* prim1, CollisionPrimitive* prim2)
{
    // Déterminer les types de primitives et appeler le détecteur approprié
    CollisionBox* box1 = dynamic_cast<CollisionBox*>(prim1);
    CollisionBox* box2 = dynamic_cast<CollisionBox*>(prim2);
    CollisionSphere* sphere1 = dynamic_cast<CollisionSphere*>(prim1);
    CollisionSphere* sphere2 = dynamic_cast<CollisionSphere*>(prim2);

    unsigned remaining = m_maxContacts - m_contactCount;
    if (remaining == 0) {
        return 0;
    }

    unsigned generated = 0;

    if (box1 && box2) {
        generated = CollisionDetector::boxAndBox(*box1, *box2, m_contacts + m_contactCount, remaining);
    }
    else if (sphere1 && sphere2) {
        generated = CollisionDetector::sphereAndSphere(*sphere1, *sphere2, m_contacts + m_contactCount);
    }

    return generated;
}

unsigned CollisionWorld::generatePlaneContacts(CollisionPrimitive* prim)
{
    unsigned totalGenerated = 0;

    for (const CollisionPlane& plane : m_planes) {
        unsigned remaining = m_maxContacts - m_contactCount;
        if (remaining == 0) {
            break;
        }

        unsigned generated = 0;

        CollisionBox* box = dynamic_cast<CollisionBox*>(prim);
        CollisionSphere* sphere = dynamic_cast<CollisionSphere*>(prim);

        if (box) {
            generated = CollisionDetector::boxAndPlane(*box, plane, m_contacts + m_contactCount, remaining);
        }
        else if (sphere) {
            generated = CollisionDetector::sphereAndPlane(*sphere, plane, m_contacts + m_contactCount);
        }

        m_contactCount += generated;
        totalGenerated += generated;
    }

    return totalGenerated;
}

unsigned CollisionWorld::detectCollisions()
{
    m_contactCount = 0;

    // Reconstruire l'octree avec les positions actuelles
    rebuildOctree();

    // Phase élargie: obtenir les paires potentielles via l'octree
    std::vector<std::pair<CollisionPrimitive*, CollisionPrimitive*>> potentialPairs;
    m_octree->getPotentialCollisions(potentialPairs);

    // Phase restreinte: tester chaque paire potentielle
    for (const auto& pair : potentialPairs) {
        if (m_contactCount >= m_maxContacts) {
            break;
        }

        unsigned generated = generateContacts(pair.first, pair.second);
        m_contactCount += generated;
    }

    // Tester les collisions avec les plans
    for (CollisionPrimitive* prim : m_primitives) {
        if (m_contactCount >= m_maxContacts) {
            break;
        }

        generatePlaneContacts(prim);
    }

    return m_contactCount;
}

void CollisionWorld::resolveCollisions(float deltaTime)
{
    if (m_contactCount > 0) {
        m_resolver->resolveContacts(m_contacts, m_contactCount, deltaTime);
    }
}
