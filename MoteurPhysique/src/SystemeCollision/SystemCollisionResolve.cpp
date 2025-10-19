#include "SystemCollisionResolve.h"

#include <cmath>

void SystemCollisionResolve::resolve(Collision collision)
{
    if (collision.p1->_inverseMasse == 0.f && collision.p2->_inverseMasse == 0.f)
        return;

    // Contact au repos → seulement séparation
    const Vector3D vitesseRelative = collision.p1->_vel - collision.p2->_vel;
    const float vitesseProjetee = vitesseRelative.produitScalaire(collision.contactNormal);
    if (std::fabs(vitesseProjetee) < 0.01f) {
        if (collision.penetration > 0.f)
            seperate(collision);
        return;
    }

    const float impulsion = -(1.0f + collision.restitution) * vitesseProjetee /
        (collision.p1->_inverseMasse + collision.p2->_inverseMasse);

    if (collision.p1->_inverseMasse > 0)
        collision.p1->_vel += collision.contactNormal * (impulsion / collision.p1->masse);
    if (collision.p2->_inverseMasse > 0)
        collision.p2->_vel -= collision.contactNormal * (impulsion / collision.p2->masse);

    if (collision.penetration > 0.f)
        seperate(collision);
}

void SystemCollisionResolve::resolveConstraint(const Collision& collision)
{
    if (collision.p1->_inverseMasse == 0.f && collision.p2->_inverseMasse == 0.f)
        return;

    const Vector3D delta = collision.p2->_pos - collision.p1->_pos;
    const float longueurActuelle = delta.norme();
    if (longueurActuelle <= 1e-4f)
        return;

    const Vector3D direction = delta / longueurActuelle;
    const float masseInverseTotale = collision.p1->_inverseMasse + collision.p2->_inverseMasse;
    if (masseInverseTotale <= 0.f)
        return;

    const float magnitude = collision.penetration / masseInverseTotale;
    const Vector3D correction = direction * magnitude;

    if (collision.p1->_inverseMasse > 0.f)
        collision.p1->_pos -= correction * collision.p1->_inverseMasse;
    if (collision.p2->_inverseMasse > 0.f)
        collision.p2->_pos += correction * collision.p2->_inverseMasse;
}

void SystemCollisionResolve::seperate(Collision collision)
{
    Vector3D deplacementP1;
    Vector3D deplacementP2;

    const float masseTotale = collision.p1->masse + collision.p2->masse;
    if (masseTotale <= 0.0f)
        return;

    const float partP1 = collision.p2->masse / masseTotale;
    const float partP2 = collision.p1->masse / masseTotale;

    deplacementP1 = collision.contactNormal * (partP1 * collision.penetration);
    deplacementP2 = collision.contactNormal * (-partP2 * collision.penetration);

    if (collision.p1->_inverseMasse > 0)
        collision.p1->_pos += deplacementP1;
    if (collision.p2->_inverseMasse > 0)
        collision.p2->_pos += deplacementP2;
}

