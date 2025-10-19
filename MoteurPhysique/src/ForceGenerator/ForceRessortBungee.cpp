#include "ForceRessortBungee.h"

void ForceRessortBungee::UpdateForce(Particule* particule, float dt)
{
    if (particule == nullptr || linkParticule == nullptr) {
        return;
    }

    if (particule->_inverseMasse <= 0.0f) {
        return;
    }

    Vector3D force = particule->_pos - linkParticule->_pos; // Vecteur entre les deux particule
    float length = force.GetNorm();

    if (length <= restLength) return;

    if (length <= 0.0f) {
        return;
    }

    float extension = length - restLength; // compressé = < restlength / étiré = > restlength

    force = force.scalar(-raideur * extension / length); // Force qui attire la particule vers l'autre

    particule->addForce(force);
}
