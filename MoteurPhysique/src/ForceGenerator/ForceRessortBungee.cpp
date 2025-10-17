#include "ForceRessortBungee.h"

void ForceRessortBungee::UpdateForce(Particule* particule, float dt)
{
    Vector3D force = particule->_pos - linkParticule->_pos; // Vecteur entre les deux particule
    float length = force.GetNorm();

    if (length <= restLength) return;
    
    float extension = length - restLength; // compressé = < restlength / étiré = > restlength

    force = force.normalize();
    force = force.scalar(-raideur * extension); // Force qui attire la particule vers l'autre

    particule->addForce(force);
}
