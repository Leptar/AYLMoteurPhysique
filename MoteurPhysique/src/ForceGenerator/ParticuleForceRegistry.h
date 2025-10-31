#pragma once
#include "ParticuleForceGenerator.h"
#include "particule.h"

#include <vector>

struct ParticuleForceRegistration {
	Particule* particule;
	ParticuleForceGenerator* fg;

	bool equal(const Particule* p, const ParticuleForceGenerator* force_generator) const {
		return particule == p && fg == force_generator;
	}
};


class ParticuleForceRegistry {
public:

	std::vector<ParticuleForceRegistration> registre;

        void add(Particule* particule, ParticuleForceGenerator* fg);
        void remove(Particule* particule, ParticuleForceGenerator* fg);
        void clear();
        void updateForces(float dt);
};
