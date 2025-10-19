#pragma once

#include "Particule.h"

enum class ForceType { Gravity, Friction };

// Interface minimale pour tout générateur de force appliqué à une particule.
class ParticuleForceGenerator {
public:
    virtual ~ParticuleForceGenerator() = default;
    virtual void UpdateForce(Particule* particule, float dt) = 0;
};

