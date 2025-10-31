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

void ParticuleForceRegistry::clear() {
        registre.clear();
}

void ParticuleForceRegistry::updateForces(float dt) {

        for (const ParticuleForceRegistration& x : registre) {
                x.fg->UpdateForce(x.particule, dt);
        }
}
