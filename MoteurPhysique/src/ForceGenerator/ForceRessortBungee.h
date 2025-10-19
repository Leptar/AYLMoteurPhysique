#pragma once
#include "ParticuleForceGenerator.h"

class ForceRessortBungee : public ParticuleForceGenerator
{
public:
    Particule* linkParticule;
    float raideur;
    float restLength;

    ForceRessortBungee(Particule* linkParticule, float raideur, float restLength)
        : linkParticule(linkParticule), raideur(raideur), restLength(restLength) {}

    void UpdateForce(Particule* particule, float dt) override;
};
