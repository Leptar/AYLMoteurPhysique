#include "ForceGravity.h"

ForceGravity::ForceGravity() {
	Gravity = Vector3D(0, 98.1f, 0);
}

void ForceGravity::UpdateForce(Particule* particule, float dt) {
        if (particule->_inverseMasse <= 0.0f) {
                return;
        }

        float masse = (particule->masse > 0.0f)
            ? particule->masse
            : 1.f / particule->_inverseMasse;

        particule->addForce(Gravity.scalar(masse));
}
