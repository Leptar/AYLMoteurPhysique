#pragma once
#include "ForceGravity.h"

class ForceFriction : public ParticuleForceGenerator {

protected:
	float coefFrictionCinetic;

public:
	ForceFriction();
	virtual void UpdateForce(Particule* particule, float dt);

};
