#pragma once
#include "Particule.h"

enum class ForceType { Gravity };
class ParticuleForceGenerator {
public:
	virtual void UpdateForce(Particule* particule, float dt) = 0;
};
