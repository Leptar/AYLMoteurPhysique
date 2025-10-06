#pragma once
#include "ParticuleForceGenerator.h"

class ForceGravity : public ParticuleForceGenerator{
public:
	Vector3D Gravity;

	ForceGravity();
	virtual void UpdateForce(Particule* particule, float dt);
};
