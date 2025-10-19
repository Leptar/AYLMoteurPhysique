#pragma once

#include "ParticuleForceGenerator.h"

// ---------------------------------------------------------------------------
// Ressort reliant deux particules libres.
// ---------------------------------------------------------------------------
class ForceRessortParticule : public ParticuleForceGenerator {
public:
    ForceRessortParticule(Particule* autre, float raideur, float longueurRepos);

    void UpdateForce(Particule* particule, float dt) override;

private:
    Particule* autreParticule;
    float k;
    float longueurRepos;
};

