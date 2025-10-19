#pragma once
#include "ForceGravity.h"

class ForceFriction : public ParticuleForceGenerator {

protected:
	float k1, k2 = 0.f;
public:

	ForceFriction();
	virtual void UpdateForce(Particule* particule, float dt);
};
