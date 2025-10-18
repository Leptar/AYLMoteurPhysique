#include "SystemCollisionResolve.h"

void SystemCollisionResolve::resolve(Collision collision)
{
    if (collision.p1->_inverseMasse == 0.f && collision.p2->_inverseMasse == 0.f)
        return;

    // Contact au repos → seulement séparation
    Vector3D Vrel = collision.p1->_vel - collision.p2->_vel;
    float vRelNorm = Vrel.dot(collision.contactNormal);
    if (fabs(vRelNorm) < 0.01f) {
        if (collision.penetration > 0.f)
            seperate(collision);
        return;
    }
    //Calcul d'impulsion instantanée
    float step = (1 + collision.restitution) * vRelNorm;
    float InstImpulse = -step/(collision.p1->_inverseMasse + collision.p2->_inverseMasse);

    //Correction de velocité (si objet pas static)
    if (collision.p1->_inverseMasse > 0)
        collision.p1->_vel = collision.p1->_vel + collision.contactNormal.scalar(InstImpulse/collision.p1->masse);
    if (collision.p2->_inverseMasse > 0)
        collision.p2->_vel = collision.p2->_vel - collision.contactNormal.scalar(InstImpulse/collision.p2->masse);

    //Correction de position
    if (collision.penetration > 0.f)
        seperate(collision);
    
}

void SystemCollisionResolve::seperate(Collision collision)
{
    Vector3D deplacementP1; //Vecteur de deplacement a appliquer pour p1
    Vector3D deplacementP2; //Vecteur de deplacement a appliquer pour p2

    // coefficient de deplacement pour p1 et p2
    float stepP1 = collision.p2->masse/(collision.p1->masse+collision.p2->masse) * collision.penetration;
    float stepP2 = -(collision.p1->masse/(collision.p1->masse+collision.p2->masse)) * collision.penetration;

    deplacementP1 = collision.contactNormal.scalar(stepP1);
    deplacementP2 = collision.contactNormal.scalar(stepP2);

    // Applique les deplacements (si peut bouger)
    if (collision.p1->_inverseMasse > 0)
        collision.p1->_pos = collision.p1->_pos + deplacementP1;
    if (collision.p2->_inverseMasse > 0)
        collision.p2->_pos = collision.p2->_pos + deplacementP2;
}
