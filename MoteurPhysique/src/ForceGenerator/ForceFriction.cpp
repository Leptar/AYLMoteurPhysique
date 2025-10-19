#include "ForceFriction.h"

#include <algorithm>

ForceFriction::ForceFriction() {}

void ForceFriction::UpdateForce(Particule* particule, float dt) {
	/*if (particule->_vel.GetNorm() != 0.f) {

		// direction
		Vector3D dir = particule->_vel.normalize();

		float normalForce = particule->AccumForce.GetNorm();
		float frictionMag = coefFrictionCinetic * normalForce;

		Vector3D Ffriction = dir.scalar(-1).scalar(frictionMag);

		particule->addForce(Ffriction);
	}*/

	if (particule->_vel.GetNorm() <= 0.f) return;

	k1 = particule->linearFriction;
	k2 = particule->quadraticFriction;

	float speed = particule->_vel.GetNorm();
	float squaredSpeed = particule->_vel.GetSquareNorm();
	float drag = k1 * speed + k2 * squaredSpeed;
	drag = std::min(drag, 0.5f * particule->masse * speed / dt); // empeche un drag absurde

	Vector3D force = particule->_vel.normalize().scalar(-drag);
	particule->addForce(force);
	
}

