#include "SystemCollisionDetection.h"

#include "SystemCollisionResolve.h"

void SystemCollisionDetection::add(Particule* p1, Particule* p2, float restitution)
{
    float penetration = p1->rayonCollision + p2->rayonCollision - (p1->_pos - p2->_pos).GetNorm();
    
    Vector3D directionCollision = (p1->_pos - p2->_pos).normalize();
    detectedCollisions.push_back(
        {p1, p2, directionCollision, penetration, restitution}
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
        SystemCollisionResolve::resolve(collision);
    }
    detectedCollisions.clear();
}

bool SystemCollisionDetection::IsColliding(Particule* p1, Particule* p2)
{
    float distance = (p1->_pos - p2->_pos).GetNorm();
    float accumRayon = p1->rayonCollision + p2->rayonCollision;

    return distance <= accumRayon;
}
