#include "CollisionDetector.h"
#include <algorithm>
#include <cmath>

float CollisionDetector::vertexPlaneDistance(const Vector3D& vertex, const CollisionPlane& plane)
{
    // Distance signée du point au plan
    return vertex.dot(plane.normal) - plane.offset;
}

unsigned CollisionDetector::boxAndPlane(const CollisionBox& box,
                                         const CollisionPlane& plane,
                                         RigidBodyContact* contacts,
                                         unsigned maxContacts)
{
    if (!box.body || maxContacts == 0) {
        return 0;
    }

    // Obtenir les 8 sommets de la boîte dans l'espace monde
    Vector3D vertices[8];
    box.getVertices(vertices);

    unsigned contactCount = 0;

    // Tester chaque sommet pour voir s'il est en collision avec le plan
    for (int i = 0; i < 8 && contactCount < maxContacts; i++) {
        float distance = vertexPlaneDistance(vertices[i], plane);

        // Si le sommet est du côté négatif du plan (collision)
        if (distance < 0) {
            // Créer un contact
            contacts[contactCount].body[0] = box.body;
            contacts[contactCount].body[1] = nullptr; // Le plan est statique
            contacts[contactCount].contactNormal = plane.normal;
            contacts[contactCount].penetration = -distance;
            contacts[contactCount].contactPoint = vertices[i] + plane.normal.scalar(distance * 0.5f);
            contacts[contactCount].restitution = 0.5f; // Coefficient de rebond par défaut
            contacts[contactCount].friction = 0.3f;

            contactCount++;
        }
    }

    return contactCount;
}

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere& sphere1,
                                             const CollisionSphere& sphere2,
                                             RigidBodyContact* contacts)
{
    if (!sphere1.body || !sphere2.body) {
        return 0;
    }

    Vector3D pos1 = sphere1.body->getPosition();
    Vector3D pos2 = sphere2.body->getPosition();

    Vector3D midline = pos1 - pos2;
    float distance = midline.GetNorm();

    // Vérifier s'il y a collision
    if (distance <= 0.0f || distance >= sphere1.radius + sphere2.radius) {
        return 0;
    }

    // Créer le contact
    Vector3D normal = midline.scalar(1.0f / distance);

    contacts[0].body[0] = sphere1.body;
    contacts[0].body[1] = sphere2.body;
    contacts[0].contactNormal = normal;
    contacts[0].penetration = sphere1.radius + sphere2.radius - distance;
    contacts[0].contactPoint = pos1 - normal.scalar(sphere1.radius - contacts[0].penetration * 0.5f);
    contacts[0].restitution = 0.5f;
    contacts[0].friction = 0.3f;

    return 1;
}

unsigned CollisionDetector::sphereAndPlane(const CollisionSphere& sphere,
                                            const CollisionPlane& plane,
                                            RigidBodyContact* contacts)
{
    if (!sphere.body) {
        return 0;
    }

    Vector3D position = sphere.body->getPosition();

    // Distance du centre de la sphère au plan
    float distance = vertexPlaneDistance(position, plane);

    // Vérifier s'il y a collision
    if (distance >= sphere.radius) {
        return 0;
    }

    // Créer le contact
    contacts[0].body[0] = sphere.body;
    contacts[0].body[1] = nullptr; // Le plan est statique
    contacts[0].contactNormal = plane.normal;
    contacts[0].penetration = sphere.radius - distance;
    contacts[0].contactPoint = position - plane.normal.scalar(distance);
    contacts[0].restitution = 0.5f;
    contacts[0].friction = 0.3f;

    return 1;
}

unsigned CollisionDetector::boxAndBox(const CollisionBox& box1,
                                       const CollisionBox& box2,
                                       RigidBodyContact* contacts,
                                       unsigned maxContacts)
{
    // Implémentation simplifiée: utiliser SAT (Separating Axis Theorem)
    // Pour cette version, on utilise une approximation simple

    if (!box1.body || !box2.body || maxContacts == 0) {
        return 0;
    }

    // Obtenir les centres des boîtes
    Vector3D center1 = box1.body->getPosition();
    Vector3D center2 = box2.body->getPosition();

    // Calculer la distance entre les centres
    Vector3D delta = center2 - center1;
    float distance = delta.GetNorm();

    // Estimation grossière: utiliser les sphères englobantes pour détecter
    float radius1 = box1.halfSize.GetNorm();
    float radius2 = box2.halfSize.GetNorm();

    if (distance >= radius1 + radius2) {
        return 0; // Pas de collision possible
    }

    // Si collision potentielle, créer un contact simplifié
    Vector3D normal = delta.GetNorm() > 0.0001f ? delta.normalize() : Vector3D(0, 1, 0);

    contacts[0].body[0] = box1.body;
    contacts[0].body[1] = box2.body;
    contacts[0].contactNormal = normal;
    contacts[0].penetration = (radius1 + radius2) - distance;
    contacts[0].contactPoint = center1 + normal.scalar(radius1 - contacts[0].penetration * 0.5f);
    contacts[0].restitution = 0.4f;
    contacts[0].friction = 0.4f;

    return 1;
}
