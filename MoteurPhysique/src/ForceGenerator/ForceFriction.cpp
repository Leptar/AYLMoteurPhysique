#include "ForceFriction.h"

#include <cmath>

ForceFriction::ForceFriction(float coefficientLineaire, float coefficientQuadratique)
    : k1(coefficientLineaire), k2(coefficientQuadratique) {}

void ForceFriction::UpdateForce(Particule* particule, float /*dt*/) {
    if (particule == nullptr || particule->estFixe()) {
        return;
    }

    const float vitesse = particule->_vel.norme();
    if (vitesse <= 0.0f) {
        return;
    }

    const float coefLineaire = (k1 > 0.0f) ? k1 : particule->linearFriction;
    const float coefQuadratique = (k2 > 0.0f) ? k2 : particule->quadraticFriction;
    const float trainee = coefLineaire * vitesse + coefQuadratique * particule->_vel.normeCarree();
    const Vector3D direction = particule->_vel.normalise();
    particule->addForce(direction * (-trainee));
}


