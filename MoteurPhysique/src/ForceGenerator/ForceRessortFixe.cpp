#include "ForceRessortFixe.h"

void ForceRessortFixe::UpdateForce(Particule* particule, float dt)
{
    Vector3D force = particule->_pos - anchreFixe; // Vecteur entre l'anchre et la particule
    float length = force.GetNorm(); 
    
    float extension = length - restLength; // compressé = < restlength / étiré = > restlength

    force = force.normalize();
    force = force.scalar(-raideur * extension); // Force qui attire la particule vers le point d'anchre

    particule->addForce(force);
}
