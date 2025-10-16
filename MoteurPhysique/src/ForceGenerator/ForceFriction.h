#pragma once
#include "ForceGravity.h"

class ForceFriction : public ParticuleForceGenerator {

protected:
	float k1, k2;
public:

	ForceFriction(float linear, float Quadratic);
	virtual void UpdateForce(Particule* particule, float dt);
};
