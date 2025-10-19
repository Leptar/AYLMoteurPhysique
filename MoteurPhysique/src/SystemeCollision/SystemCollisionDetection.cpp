#include "SystemCollisionDetection.h"

#include <cmath>

#include "SystemCollisionResolve.h"

namespace {
constexpr float kLengthEpsilon = 1e-5f;
}

void SystemCollisionDetection::add(Particule* p1, Particule* p2, float restitution, CollisionType type)
{
    if (p1 == nullptr || p2 == nullptr) {
        return;
    }

    Vector3D delta = p1->_pos - p2->_pos;
    float distance = delta.GetNorm();
    float penetration = p1->rayonCollision + p2->rayonCollision - distance;

    if (penetration <= kLengthEpsilon) {
        return;
    }

    Vector3D directionCollision(0.0f, 1.0f, 0.0f);
    if (distance > kLengthEpsilon) {
        directionCollision = delta.scalar(1.0f / distance);
    }

    detectedCollisions.push_back(
        {p1, p2, directionCollision, penetration, restitution, type}
        );
}

void SystemCollisionDetection::remove(const Collision& collision)
{
    for (auto it = detectedCollisions.begin(); it != detectedCollisions.end(); ++it) {
        if (it->equal(collision.p1, collision.p2)) {
            detectedCollisions.erase(it);
            break;
        }
    }
}

bool SystemCollisionDetection::clear()
{
   if (detectedCollisions.empty()) {
       return false;
   }
   detectedCollisions.clear();
   return true;
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
    if (p1 == nullptr || p2 == nullptr) {
        return false;
    }

    float distance = (p1->_pos - p2->_pos).GetNorm();
    float accumRayon = p1->rayonCollision + p2->rayonCollision;

    return distance <= accumRayon + kLengthEpsilon;
}

void SystemCollisionDetection::addRodConstraint(Particule* p1, Particule* p2, float length)
{
    if (p1 == nullptr || p2 == nullptr) {
        return;
    }

    Vector3D delta = p2->_pos - p1->_pos;
    float currentLength = delta.GetNorm();

    if (currentLength <= kLengthEpsilon) {
        return;
    }

    float lengthDelta = currentLength - length;
    if (std::fabs(lengthDelta) <= kLengthEpsilon) {
        return; // pas de collision, la tige est à sa longueur idéale
    }

    Collision c;
    c.p1 = p1;
    c.p2 = p2;
    c.type = CollisionType::Rod;
    c.restitution = 0.0f;

    if (lengthDelta > 0.0f) {
        c.contactNormal = delta.scalar(1.0f / currentLength);
        c.penetration = lengthDelta;
    } else {
        c.contactNormal = delta.scalar(-1.0f / currentLength);
        c.penetration = -lengthDelta;
    }

    detectedCollisions.push_back(c);

}

void SystemCollisionDetection::addCableConstraint(Particule* p1, Particule* p2, float maxLength, float restitution)
{
    if (p1 == nullptr || p2 == nullptr) {
        return;
    }

    Vector3D delta = p2->_pos - p1->_pos;
    float currentLength = delta.GetNorm();

    if (currentLength <= kLengthEpsilon) {
        return;
    }

    if (currentLength <= maxLength + kLengthEpsilon)
        return; // le câble est détendu, aucune collision

    Collision c;
    c.p1 = p1;
    c.p2 = p2;
    c.type = CollisionType::Cable;
    c.restitution = restitution;
    c.contactNormal = delta.scalar(1.0f / currentLength);
    c.penetration = currentLength - maxLength;

    detectedCollisions.push_back(c);
}
