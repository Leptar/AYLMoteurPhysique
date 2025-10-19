#pragma once

#include "ParticuleForceGenerator.h"

// ---------------------------------------------------------------------------
// Générateur appliquant la gravité uniforme sur une particule.
// ---------------------------------------------------------------------------
class ForceGravity : public ParticuleForceGenerator {
public:
    ForceGravity();

    void definirGravite(const Vector3D& nouvelleGravite);
    void UpdateForce(Particule* particule, float dt) override;

private:
    Vector3D gravite;
};

