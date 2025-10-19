#include "ForceRessortFixe.h"

namespace {
constexpr float kDistanceEpsilon = 1e-5f;
}

ForceRessortFixe::ForceRessortFixe(const Vector3D& ancre, float raideur, float longueurRepos)
    : pointAncrage(ancre), k(raideur), longueurRepos(longueurRepos) {}

void ForceRessortFixe::UpdateForce(Particule* particule, float /*dt*/)
{
    if (particule == nullptr || particule->estFixe()) {
        return;
    }

    Vector3D direction = particule->_pos - pointAncrage;
    const float distance = direction.norme();

    if (distance <= kDistanceEpsilon) {
        return;
    }

    const float extension = distance - longueurRepos;
    const Vector3D force = direction * (-k * extension / distance);

    particule->addForce(force);
}

