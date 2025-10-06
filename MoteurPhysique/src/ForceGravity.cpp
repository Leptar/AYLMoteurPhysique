#include "ForceGravity.h"

ForceGravity::ForceGravity() {
	Gravity = Vector3D(0, 98.1f, 0);
}

void ForceGravity::UpdateForce(Particule* particule, float dt) {
	float masse = 1.f/particule->_inverseMasse;
	if (masse != 0.0f) {
		particule->addForce(Gravity.scalar(masse));
	}
}
