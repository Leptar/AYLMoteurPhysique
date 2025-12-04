#include "SystemeCollisionDetection.h"
#include <algorithm>

void SystemeCollisionDetection::add(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, CollisionType type)
{
    if (!p1 || !p2) return;

    Contact contact;
    // Conversion : On récupère les CorpsRigides associés aux primitives via le membre 'corpsRigide'
    contact.c1 = p1->corpsRigide; 
    contact.c2 = p2->corpsRigide;
    
    contact.contactPoint = point;
    contact.contactNormal = normal;
    contact.penetration = penetration;
    contact.restitution = restitution;
    
    // La structure Contact a un champ friction, mais la méthode add ne le passait pas.
    // On met une valeur par défaut raisonnable.
    contact.friction = 0.5f; 

    // Note : Si tu veux stocker le CollisionType, il faudra l'ajouter dans la struct Contact.
    // contact.type = type; 

    detectedCollisions.push_back(contact);
}

void SystemeCollisionDetection::addPlane(Primitive* p1, Primitive* p2, const Vector3D& point, const Vector3D& normal, float penetration, float restitution, CollisionType type)
{
    if (!p1 || !p2) return;

    Contact contact;
    contact.c1 = p1->corpsRigide;
    contact.c2 = p2->corpsRigide;
    
    contact.contactPoint = point;
    contact.contactNormal = normal;
    contact.penetration = penetration;
    contact.restitution = restitution;
    contact.friction = 0.5f; // Valeur par défaut

    // contact.type = type;

    detectedCollisions.push_back(contact);
}

void SystemeCollisionDetection::remove(const Contact& contact)
{
    // On retire la collision qui correspond aux corps rigides donnés
    detectedCollisions.erase(
        std::remove_if(detectedCollisions.begin(), detectedCollisions.end(),
            [&](const Contact& c) { 
                // On utilise la méthode equal de la struct Contact qui compare c1 et c2
                return c.equal(contact.c1, contact.c2); 
            }),
        detectedCollisions.end());
}

bool SystemeCollisionDetection::empty() const
{
    return detectedCollisions.empty();
}

void SystemeCollisionDetection::clear()
{
    detectedCollisions.clear();
}

void SystemeCollisionDetection::resolveAll()
{
    // À IMPLEMENTER : RÉSOLUTION DES CONTACTS (IMPULSIONS ANGULAIRES ET LINÉAIRES)
    // On parcourt detectedCollisions et on applique les impulsions sur c1 et c2
}

std::size_t SystemeCollisionDetection::count() const
{
    return detectedCollisions.size();
}

bool SystemeCollisionDetection::IsColliding(Primitive* p1, Primitive* p2)
{
    if (!p1 || !p2) return false;
    return false; 
}

void SystemeCollisionDetection::addRodConstraint(Primitive* p1, Primitive* p2, float length)
{
    if (!p1 || !p2) return;

    Contact contact;
    contact.c1 = p1->corpsRigide;
    contact.c2 = p2->corpsRigide;
    
    // Note : Sans le champ 'type' dans Contact, le résolveur ne saura pas que c'est une tige.
    // contact.type = CollisionType::Rod;
    
    contact.friction = 0.0f;
    contact.restitution = 0.0f;

    detectedCollisions.push_back(contact);
}

void SystemeCollisionDetection::addCableConstraint(Primitive* p1, Primitive* p2, float maxLength, float restitution)
{
    if (!p1 || !p2) return;

    Contact contact;
    contact.c1 = p1->corpsRigide;
    contact.c2 = p2->corpsRigide;
    
    contact.restitution = restitution;
    contact.friction = 0.0f;
    
    // contact.type = CollisionType::Cable;

    detectedCollisions.push_back(contact);
}

// -----------------------------------------------------------------------
// IMPLÉMENTATION BOÎTE - PLAN qui va généraliser presque tous les contacts possibles
// -----------------------------------------------------------------------
void SystemeCollisionDetection::DetectBoxPlane(Box* box, Plane* plane)
{
    // 1. Matrices de transformation
    Matrix4 boxToWorld = box->GetTransformMatrix();
    Matrix4 planeToWorld = plane->GetTransformMatrix();

    // 2. Calcul de la normale du plan en World Space
    Vector3D localOrigin(0, 0, 0);
    Vector3D worldOrigin = planeToWorld * localOrigin;
    
    Vector3D normalPosLocal = plane->normal; 
    Vector3D normalPosWorld = planeToWorld * normalPosLocal;
    
    Vector3D planeNormalWorld = normalPosWorld - worldOrigin;
    planeNormalWorld = planeNormalWorld.normalize();

    // 3. Point de référence sur le plan (P)
    Vector3D pointOnPlaneLocal = plane->normal.scalar(plane->PlaneOffset);
    Vector3D pointOnPlaneWorld = planeToWorld * pointOnPlaneLocal;

    // 4. Les 8 sommets de la boîte en Local
    float hx = box->HalfExtent.x;
    float hy = box->HalfExtent.y;
    float hz = box->HalfExtent.z;

    std::vector<Vector3D> verticesLocal = {
        Vector3D( hx,  hy,  hz), Vector3D(-hx,  hy,  hz),
        Vector3D( hx, -hy,  hz), Vector3D(-hx, -hy,  hz),
        Vector3D( hx,  hy, -hz), Vector3D(-hx,  hy, -hz),
        Vector3D( hx, -hy, -hz), Vector3D(-hx, -hy, -hz)
    };

    // 5. Test intersection
    for (const auto& vertexLocal : verticesLocal)
    {
        // Q: Sommet en World
        Vector3D vertexWorld = boxToWorld * vertexLocal;

        // t = n . (Q - P)
        Vector3D Q_minus_P = vertexWorld - pointOnPlaneWorld;
        float t = planeNormalWorld.dot(Q_minus_P);

        // Si t <= 0, collision
        if (t <= 0)
        {
            // Calcul du point de contact R = Q - t*n
            Vector3D displacement = planeNormalWorld.scalar(t);
            Vector3D contactPoint = vertexWorld - displacement;

            // Ajout via addPlane
            addPlane(box, plane, contactPoint, planeNormalWorld, -t, 0.5f, CollisionType::Contact);
        }
    }
}