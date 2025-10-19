#pragma once

#include <vector>

#include "Particule.h"
#include "ParticuleForceGenerator.h"

// Association entre une particule et le générateur de force qui lui est lié.
struct ParticuleForceRegistration {
    Particule* particule = nullptr;
    ParticuleForceGenerator* generateur = nullptr;

    [[nodiscard]] bool correspond(const Particule* p, const ParticuleForceGenerator* g) const {
        return particule == p && generateur == g;
    }
};

// Gère la mise à jour de toutes les forces enregistrées.
class ParticuleForceRegistry {
public:
    void add(Particule* particule, ParticuleForceGenerator* fg);
    void remove(Particule* particule, ParticuleForceGenerator* fg);
    bool clear();
    void updateForces(float dt);

private:
    std::vector<ParticuleForceRegistration> registre;
};

