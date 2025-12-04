#include "SystemCollisionDetection.h"

#include "SystemCollisionResolve.h"

void SystemCollisionDetection::add(Particule* p1, Particule* p2, float restitution, collision_type type)
{
    if (p1 == nullptr || p2 == nullptr)
        return;

    float penetration = p1->rayonCollision + p2->rayonCollision - (p1->_pos - p2->_pos).GetNorm();
    if (penetration <= 0.f)
        return;

    Vector3D directionCollision = (p1->_pos - p2->_pos).normalize();
    detectedCollisions.push_back({p1, p2, directionCollision, penetration, restitution, type});
}

void SystemCollisionDetection::addPlane(Particule* p1, const Vector3D& normal, float penetration, float restitution, collision_type type)
{
    if (p1 == nullptr || penetration <= 0.f)
        return;

    Collision collision;
    collision.p1 = p1;
    collision.p2 = nullptr;
    collision.contactNormal = normal;
    collision.penetration = penetration;
    collision.restitution = restitution;
    collision.type = type;

    detectedCollisions.push_back(collision);
}

void SystemCollisionDetection::remove(const Collision& collision)
{
    for (auto it = detectedCollisions.begin(); it != detectedCollisions.end(); it++) {
        if (it->equal(collision.p1, collision.p2)) {
            detectedCollisions.erase(it);
        }
    }
}

void SystemCollisionDetection::resolveAll()
{
    for (const Collision& collision : detectedCollisions)
    {
        switch (collision.type)
        {
        case collision_type::Contact:
        case collision_type::Resting:
            SystemCollisionResolve::resolve(collision);
            break;

        case collision_type::Cable:
        case collision_type::Rod:
            SystemCollisionResolve::resolveConstraint(collision);
            break;
        }
    }

    detectedCollisions.clear();
}

bool SystemCollisionDetection::empty() const
{
    return detectedCollisions.empty();
}

void SystemCollisionDetection::clear()
{
    detectedCollisions.clear();
}

std::size_t SystemCollisionDetection::count() const
{
    return detectedCollisions.size();
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
    c.type = collision_type::Rod;
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
    c.type = collision_type::Cable;
    c.restitution = restitution;
    c.contactNormal = delta.normalize();
    c.penetration = currentLength - maxLength;

    detectedCollisions.push_back(c);
}
