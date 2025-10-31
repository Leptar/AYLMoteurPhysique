#pragma once
#include "Particule.h"

enum class ForceType {
    Gravity,
    Friction,
    SpringFixed,
    SpringParticle,
    SpringBungee
};

class ParticuleForceGenerator {
public:
	virtual ~ParticuleForceGenerator() = default;
	virtual void UpdateForce(Particule* particule, float dt) = 0;
};
