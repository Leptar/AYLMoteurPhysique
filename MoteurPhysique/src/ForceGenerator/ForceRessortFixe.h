#pragma once
#include "ParticuleForceGenerator.h"

class ForceRessortFixe : public ParticuleForceGenerator
{
public:
    Vector3D anchreFixe;
    float raideur;
    float restLength;

    ForceRessortFixe(Vector3D anchor, float radius, float restLength)
        : anchreFixe(anchor), raideur(radius), restLength(restLength) {}

    void UpdateForce(Particule* particule, float dt) override;
};
