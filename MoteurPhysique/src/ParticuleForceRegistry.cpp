#include "ParticuleForceRegistry.h"

void ParticuleForceRegistry::add(Particule* particule, ParticuleForceGenerator* fg) {
	registre.push_back({particule, fg});
}

void ParticuleForceRegistry::remove(Particule* particule, ParticuleForceGenerator* fg) {
	for (auto it = registre.begin(); it != registre.end(); it++) {
		if (it->particule == particule && it->fg == fg) {
			registre.erase(it);
		}
	}
}

bool ParticuleForceRegistry::clear() {
	return registre.empty();
}

void ParticuleForceRegistry::updateForces(float dt) {

	for (ParticuleForceRegistration x : registre) {
		x.fg->UpdateForce(x.particule, dt);
	}
}
