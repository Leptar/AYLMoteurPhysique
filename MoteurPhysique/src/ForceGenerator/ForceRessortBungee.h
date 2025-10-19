#pragma once

#include "ParticuleForceGenerator.h"

// ---------------------------------------------------------------------------
// Ressort de type élastique qui ne pousse qu'en traction (bungee).
// ---------------------------------------------------------------------------
class ForceRessortBungee : public ParticuleForceGenerator {
public:
    ForceRessortBungee(Particule* autre, float raideur, float longueurRepos);

    void UpdateForce(Particule* particule, float dt) override;

private:
    Particule* autreParticule;
    float k;
    float longueurRepos;
};

