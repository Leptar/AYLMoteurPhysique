#include "SystemCollisionResolve.h"

#include <cmath>

void SystemCollisionResolve::resolve(const Collision& collision)
{
    float inverseMassP1 = collision.p1 ? collision.p1->_inverseMasse : 0.f;
    float inverseMassP2 = collision.p2 ? collision.p2->_inverseMasse : 0.f;

    if (inverseMassP1 == 0.f && inverseMassP2 == 0.f)
        return;

    Vector3D velocityP1 = collision.p1 ? collision.p1->_vel : Vector3D();
    Vector3D velocityP2 = collision.p2 ? collision.p2->_vel : Vector3D();
    Vector3D relativeVelocity = velocityP1 - velocityP2;
    float vRelNorm = relativeVelocity.dot(collision.contactNormal);

    if (std::fabs(vRelNorm) < 0.01f) {
        // La vitesse relative est trop faible pour justifier une impulsion ; on se contente de corriger la pénétration.
        if (collision.penetration > 0.f)
            seperate(collision);
        return;
    }

    float totalInverseMass = inverseMassP1 + inverseMassP2;
    if (totalInverseMass <= 0.f)
        return;

    // Calcule l'impulsion qui inversera la vitesse relative le long de la normale.
    float impulseScalar = -(1.f + collision.restitution) * vRelNorm;
    impulseScalar /= totalInverseMass;

    Vector3D impulse = collision.contactNormal.scalar(impulseScalar);

    if (collision.p1 && inverseMassP1 > 0.f)
        collision.p1->_vel = collision.p1->_vel + impulse.scalar(inverseMassP1);
    if (collision.p2 && inverseMassP2 > 0.f)
        collision.p2->_vel = collision.p2->_vel - impulse.scalar(inverseMassP2);

    if (collision.penetration > 0.f)
        seperate(collision);
}

void SystemCollisionResolve::resolveConstraint(const Collision& collision)
{
    if (collision.p1 == nullptr || collision.p2 == nullptr)
        return;

    if (collision.p1->_inverseMasse == 0.f && collision.p2->_inverseMasse == 0.f)
        return;

    Vector3D delta = collision.p2->_pos - collision.p1->_pos;
    float currentLength = delta.GetNorm();

    if (currentLength <= 0.0001f)
        return;

    Vector3D direction = delta.normalize();

    float totalInverseMass = collision.p1->_inverseMasse + collision.p2->_inverseMasse;
    if (totalInverseMass <= 0.f)
        return;

    // Projette les deux particules le long de l'axe de la contrainte pour éliminer la pénétration.
    float correctionMagnitude = collision.penetration / totalInverseMass;
    Vector3D correction = direction.scalar(correctionMagnitude);

    if (collision.p1->_inverseMasse > 0.f)
        collision.p1->_pos = collision.p1->_pos - correction.scalar(collision.p1->_inverseMasse);
    if (collision.p2->_inverseMasse > 0.f)
        collision.p2->_pos = collision.p2->_pos + correction.scalar(collision.p2->_inverseMasse);
}

void SystemCollisionResolve::seperate(const Collision& collision)
{
    float inverseMassP1 = collision.p1 ? collision.p1->_inverseMasse : 0.f;
    float inverseMassP2 = collision.p2 ? collision.p2->_inverseMasse : 0.f;
    float totalInverseMass = inverseMassP1 + inverseMassP2;

    if (totalInverseMass <= 0.f)
        return;

    Vector3D correction = collision.contactNormal.scalar(collision.penetration / totalInverseMass);

    if (collision.p1 && inverseMassP1 > 0.f)
        collision.p1->_pos = collision.p1->_pos + correction.scalar(inverseMassP1);
    if (collision.p2 && inverseMassP2 > 0.f)
        collision.p2->_pos = collision.p2->_pos - correction.scalar(inverseMassP2);
}
