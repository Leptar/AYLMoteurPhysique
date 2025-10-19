#pragma once

#include "ParticuleForceGenerator.h"

// ---------------------------------------------------------------------------
// Générateur de force de frottement combinant termes linéaire et quadratique.
// ---------------------------------------------------------------------------
class ForceFriction : public ParticuleForceGenerator {
public:
    ForceFriction(float coefficientLineaire, float coefficientQuadratique);

    void UpdateForce(Particule* particule, float dt) override;

private:
    float k1;
    float k2;
};

