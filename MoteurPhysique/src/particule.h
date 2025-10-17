#ifndef PARTICULE_H
#define PARTICULE_H

#include "3DVector.h"

class Particule {
public:
    Vector3D _oldPos,_pos, _vel, _acc, AccumForce;

    float _inverseMasse, linearFriction, quadraticFriction;

    Particule(Vector3D pos,
              Vector3D vel,
              Vector3D acc,
              float masse = 1.0,
              float linear = 0.0f,
              float quadratic = 0.0f);

    void integrerEuler(float dt);
    void integrerVerlet(float dt);

	void addForce(const Vector3D& Force);
	void clearForce();
};

#endif
