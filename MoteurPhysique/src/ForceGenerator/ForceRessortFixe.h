#pragma once

#include "ParticuleForceGenerator.h"

// ---------------------------------------------------------------------------
// Ressort reliant une particule à un point fixe de l'espace.
// ---------------------------------------------------------------------------
class ForceRessortFixe : public ParticuleForceGenerator {
public:
    ForceRessortFixe(const Vector3D& ancre, float raideur, float longueurRepos);

    void UpdateForce(Particule* particule, float dt) override;

private:
    Vector3D pointAncrage;
    float k;
    float longueurRepos;
};

