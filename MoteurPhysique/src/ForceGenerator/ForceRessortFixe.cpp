#include "ForceRessortFixe.h"

void ForceRessortFixe::UpdateForce(Particule* particule, float dt)
{
    if (particule == nullptr) {
        return;
    }

    if (particule->_inverseMasse <= 0.0f) {
        return;
    }

    Vector3D force = particule->_pos - anchreFixe; // Vecteur entre l'anchre et la particule
    float length = force.GetNorm();

    if (length <= 0.0f) {
        return;
    }

    float extension = length - restLength; // compressé = < restlength / étiré = > restlength

    force = force.scalar(-raideur * extension / length); // Force qui attire la particule vers le point d'anchre

    particule->addForce(force);
}
