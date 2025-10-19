#include "ForceGravity.h"

ForceGravity::ForceGravity() : gravite(0.0f, 98.1f, 0.0f) {}

void ForceGravity::definirGravite(const Vector3D& nouvelleGravite) {
    gravite = nouvelleGravite;
}

void ForceGravity::UpdateForce(Particule* particule, float /*dt*/) {
    if (particule == nullptr || particule->estFixe()) {
        return;
    }

    const float masse = 1.0f / particule->_inverseMasse;
    particule->addForce(gravite * masse);
}

