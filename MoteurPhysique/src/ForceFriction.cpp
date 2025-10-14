#include "ForceFriction.h"

ForceFriction::ForceFriction() {
	coefFrictionCinetic = 0.0f;
}

void ForceFriction::UpdateForce(Particule * particule, float dt) {
	if (particule->_vel.GetNorm() != 0.f) {

		// direction
		Vector3D dir = particule->_vel.normalize();

		float normalForce = particule->AccumForce.GetNorm();
		float frictionMag = coefFrictionCinetic * normalForce;

		Vector3D Ffriction = dir.scalar(-1).scalar(frictionMag);

		particule->addForce(Ffriction);
	}
}
