#include "ParticuleForceRegistry.h"

#include <algorithm>

void ParticuleForceRegistry::add(Particule* particule, ParticuleForceGenerator* fg) {
    if (particule == nullptr || fg == nullptr) {
        return;
    }
    registre.push_back({particule, fg});
}

void ParticuleForceRegistry::remove(Particule* particule, ParticuleForceGenerator* fg) {
    registre.erase(
        std::remove_if(
            registre.begin(),
            registre.end(),
            [particule, fg](const ParticuleForceRegistration& registration) {
                return registration.correspond(particule, fg);
            }),
        registre.end());
}

bool ParticuleForceRegistry::clear() {
    if (registre.empty()) {
        return false;
    }
    registre.clear();
    return true;
}

void ParticuleForceRegistry::updateForces(float dt) {
    for (const ParticuleForceRegistration& registration : registre) {
        if (registration.generateur != nullptr && registration.particule != nullptr) {
            registration.generateur->UpdateForce(registration.particule, dt);
        }
    }
}

