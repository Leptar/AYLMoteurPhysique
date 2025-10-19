#include "SystemCollisionDetection.h"

#include "SystemCollisionResolve.h"

void SystemCollisionDetection::add(Particule* p1, Particule* p2, float restitution, CollisionType type)
{
    float penetration = p1->rayonCollision + p2->rayonCollision - (p1->_pos - p2->_pos).GetNorm();
    
    Vector3D directionCollision = (p1->_pos - p2->_pos).normalize();
    detectedCollisions.push_back(
        {p1, p2, directionCollision, penetration, restitution, type}
        );
}

void SystemCollisionDetection::remove(const Collision& collision)
{
    for (auto it = detectedCollisions.begin(); it != detectedCollisions.end(); it++) {
        if (it->equal(collision.p1, collision.p2)) {
            detectedCollisions.erase(it);
        }
    }
}

bool SystemCollisionDetection::clear()
{
   return detectedCollisions.empty();
}

void SystemCollisionDetection::resolveAll()
{
    for (Collision collision : detectedCollisions)
    {
        switch (collision.type)
        {
        case CollisionType::Contact:
        case CollisionType::Resting:
            SystemCollisionResolve::resolve(collision);
            break;

        case CollisionType::Cable:
        case CollisionType::Rod:
            SystemCollisionResolve::resolveConstraint(collision);
            break;
        }
    }
}

bool SystemCollisionDetection::IsColliding(Particule* p1, Particule* p2)
{
    float distance = (p1->_pos - p2->_pos).GetNorm();
    float accumRayon = p1->rayonCollision + p2->rayonCollision;

    return distance <= accumRayon;
}

void SystemCollisionDetection::addRodConstraint(Particule* p1, Particule* p2, float length)
{
    Vector3D delta = p2->_pos - p1->_pos;
    float currentLength = delta.GetNorm();

    if (currentLength == length)
        return; // pas de collision, la tige est à sa longueur idéale

    Collision c;
    c.p1 = p1;
    c.p2 = p2;
    c.type = CollisionType::Rod;
    c.restitution = 0.0f;

    if (currentLength > length) {
        c.contactNormal = delta.normalize();
        c.penetration = currentLength - length;
    } else {
        c.contactNormal = delta.normalize().scalar(-1);
        c.penetration = length - currentLength;
    }

    detectedCollisions.push_back(c);

}

void SystemCollisionDetection::addCableConstraint(Particule* p1, Particule* p2, float maxLength, float restitution)
{
    Vector3D delta = p2->_pos - p1->_pos;
    float currentLength = delta.GetNorm();

    if (currentLength < maxLength)
        return; // le câble est détendu, aucune collision

    Collision c;
    c.p1 = p1;
    c.p2 = p2;
    c.type = CollisionType::Cable;
    c.restitution = restitution;
    c.contactNormal = delta.normalize();
    c.penetration = currentLength - maxLength;

    detectedCollisions.push_back(c);
}
