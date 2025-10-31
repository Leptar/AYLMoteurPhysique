#pragma once
#include "ParticuleForceGenerator.h"

class ForceRessortParticule : public ParticuleForceGenerator
{
public:
    Particule* linkParticule;
    float raideur;
    float restLength;

    ForceRessortParticule(Particule* linkParticule, float raideur, float restLength)
        : linkParticule(linkParticule), raideur(raideur), restLength(restLength) {}

    void UpdateForce(Particule* particule, float dt) override;
};
