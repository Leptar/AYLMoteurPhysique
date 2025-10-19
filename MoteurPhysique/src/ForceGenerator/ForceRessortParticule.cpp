#include "ForceRessortParticule.h"

namespace {
constexpr float kDistanceEpsilon = 1e-5f;
}

ForceRessortParticule::ForceRessortParticule(Particule* autre, float raideur, float longueurRepos)
    : autreParticule(autre), k(raideur), longueurRepos(longueurRepos) {}

void ForceRessortParticule::UpdateForce(Particule* particule, float /*dt*/)
{
    if (particule == nullptr || autreParticule == nullptr || particule->estFixe()) {
        return;
    }

    Vector3D direction = particule->_pos - autreParticule->_pos;
    const float distance = direction.norme();

    if (distance <= kDistanceEpsilon) {
        return;
    }

    const float extension = distance - longueurRepos;
    const Vector3D force = direction * (-k * extension / distance);

    particule->addForce(force);
}

