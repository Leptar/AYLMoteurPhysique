#include "ParticuleForceRegistry.h"

void ParticuleForceRegistry::add(Particule* particule, ParticuleForceGenerator* fg) {
	registre.push_back({particule, fg});
}

void ParticuleForceRegistry::remove(Particule* particule, ParticuleForceGenerator* fg) {
        for (auto it = registre.begin(); it != registre.end(); ++it) {
                if (it->particule == particule && it->fg == fg) {
                        registre.erase(it);
                        break;
                }
        }
}

bool ParticuleForceRegistry::clear() {
        if (registre.empty()) {
                return false;
        }
        registre.clear();
        return true;
}

void ParticuleForceRegistry::updateForces(float dt) {

        for (const ParticuleForceRegistration& x : registre) {
                if (x.fg != nullptr && x.particule != nullptr) {
                        x.fg->UpdateForce(x.particule, dt);
                }
        }
}
