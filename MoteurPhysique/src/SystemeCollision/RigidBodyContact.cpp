#include "RigidBodyContact.h"
#include <cmath>
#include <algorithm>

float RigidBodyContact::calculateSeparatingVelocity() const
{
    Vector3D relativeVelocity = body[0]->getVelocite();

    if (body[1]) {
        relativeVelocity = relativeVelocity - body[1]->getVelocite();
    }

    // Ajouter les composantes de vitesse angulaire
    Vector3D relContactPos = contactPoint - body[0]->getPosition();
    Vector3D angularVel = body[0]->getVelociteAngulaire().cross(relContactPos);
    relativeVelocity = relativeVelocity + angularVel;

    if (body[1]) {
        Vector3D relContactPos2 = contactPoint - body[1]->getPosition();
        Vector3D angularVel2 = body[1]->getVelociteAngulaire().cross(relContactPos2);
        relativeVelocity = relativeVelocity - angularVel2;
    }

    return relativeVelocity.dot(contactNormal);
}

void RigidBodyContact::resolveVelocity(float deltaTime)
{
    // Calculer la vitesse de séparation
    float separatingVelocity = calculateSeparatingVelocity();

    // Si les corps s'éloignent déjà, ne rien faire
    if (separatingVelocity > 0) {
        return;
    }

    // Calculer la nouvelle vitesse de séparation
    float newSepVelocity = -separatingVelocity * restitution;

    // Calculer la variation de vitesse totale nécessaire
    float deltaVelocity = newSepVelocity - separatingVelocity;

    // Calculer la masse effective au point de contact
    float totalInverseMass = body[0]->getInverseMasse();
    if (body[1]) {
        totalInverseMass += body[1]->getInverseMasse();
    }

    // Ajouter les contributions rotationnelles
    Vector3D relContactPos = contactPoint - body[0]->getPosition();
    Vector3D torquePerUnitImpulse = relContactPos.cross(contactNormal);
    Vector3D rotationPerUnitImpulse =
        body[0]->getInverseInertiaTensorWorld().transform(torquePerUnitImpulse);
    Vector3D velocityPerUnitImpulse = rotationPerUnitImpulse.cross(relContactPos);
    float angularComponent = velocityPerUnitImpulse.dot(contactNormal);
    totalInverseMass += angularComponent;

    if (body[1]) {
        Vector3D relContactPos2 = contactPoint - body[1]->getPosition();
        Vector3D torquePerUnitImpulse2 = relContactPos2.cross(contactNormal);
        Vector3D rotationPerUnitImpulse2 =
            body[1]->getInverseInertiaTensorWorld().transform(torquePerUnitImpulse2);
        Vector3D velocityPerUnitImpulse2 = rotationPerUnitImpulse2.cross(relContactPos2);
        float angularComponent2 = velocityPerUnitImpulse2.dot(contactNormal);
        totalInverseMass += angularComponent2;
    }

    // Calculer l'impulsion nécessaire
    float impulse = deltaVelocity / totalInverseMass;
    Vector3D impulsePerIMass = contactNormal.scalar(impulse);

    // Appliquer l'impulsion au premier corps
    Vector3D newVelocity = body[0]->getVelocite() + impulsePerIMass.scalar(body[0]->getInverseMasse());
    body[0]->setVelocite(newVelocity);

    Vector3D impulsiveTorque = relContactPos.cross(impulsePerIMass);
    Vector3D rotationChange = body[0]->getInverseInertiaTensorWorld().transform(impulsiveTorque);
    Vector3D newAngularVelocity = body[0]->getVelociteAngulaire() + rotationChange;
    body[0]->setVelociteAngulaire(newAngularVelocity);

    // Appliquer l'impulsion au second corps (s'il existe)
    if (body[1]) {
        Vector3D newVelocity2 = body[1]->getVelocite() - impulsePerIMass.scalar(body[1]->getInverseMasse());
        body[1]->setVelocite(newVelocity2);

        Vector3D relContactPos2 = contactPoint - body[1]->getPosition();
        Vector3D impulsiveTorque2 = relContactPos2.cross(impulsePerIMass.scalar(-1.0f));
        Vector3D rotationChange2 = body[1]->getInverseInertiaTensorWorld().transform(impulsiveTorque2);
        Vector3D newAngularVelocity2 = body[1]->getVelociteAngulaire() + rotationChange2;
        body[1]->setVelociteAngulaire(newAngularVelocity2);
    }
}

void RigidBodyContact::resolveInterpenetration(float deltaTime)
{
    // Si pas d'interpénétration, ne rien faire
    if (penetration <= 0) {
        return;
    }

    // Calculer la masse inverse totale
    float totalInverseMass = body[0]->getInverseMasse();
    if (body[1]) {
        totalInverseMass += body[1]->getInverseMasse();
    }

    // Si les deux corps sont immobiles, ne rien faire
    if (totalInverseMass <= 0) {
        return;
    }

    // Calculer le déplacement par unité de masse inverse
    Vector3D movePerIMass = contactNormal.scalar(penetration / totalInverseMass);

    // Déplacer le premier corps
    Vector3D movement1 = movePerIMass.scalar(body[0]->getInverseMasse());
    Vector3D newPos1 = body[0]->getPosition() + movement1;
    body[0]->setPosition(newPos1);

    // Déplacer le second corps (s'il existe)
    if (body[1]) {
        Vector3D movement2 = movePerIMass.scalar(-body[1]->getInverseMasse());
        Vector3D newPos2 = body[1]->getPosition() + movement2;
        body[1]->setPosition(newPos2);
    }
}

void RigidBodyContact::resolve(float deltaTime)
{
    resolveVelocity(deltaTime);
    resolveInterpenetration(deltaTime);
}

// ContactResolver implementation

ContactResolver::ContactResolver(unsigned iterations)
    : m_iterations(iterations), m_iterationsUsed(0)
{
}

void ContactResolver::setIterations(unsigned iterations)
{
    m_iterations = iterations;
}

void ContactResolver::resolveContacts(RigidBodyContact* contacts, unsigned numContacts, float deltaTime)
{
    m_iterationsUsed = 0;

    while (m_iterationsUsed < m_iterations)
    {
        // Trouver le contact avec la plus grande vitesse de fermeture
        float max = 0.0f;
        unsigned maxIndex = numContacts;

        for (unsigned i = 0; i < numContacts; i++)
        {
            float sepVel = contacts[i].calculateSeparatingVelocity();
            if (sepVel < max)
            {
                max = sepVel;
                maxIndex = i;
            }
        }

        // Si tous les contacts se séparent déjà, arrêter
        if (maxIndex == numContacts) {
            break;
        }

        // Résoudre le contact le plus critique
        contacts[maxIndex].resolve(deltaTime);

        m_iterationsUsed++;
    }
}
