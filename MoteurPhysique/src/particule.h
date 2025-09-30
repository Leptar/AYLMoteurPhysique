#ifndef PARTICULE_H
#define PARTICULE_H

#include "3DVector.h"
#include <cmath>

class Particule {
    Vector3D _oldPos;

    Vector3D _pos;

    Vector3D _vel;

    Vector3D _force;

    float _inverseMasse;

public:
    Particule(Vector3D pos,
              Vector3D vel,
              Vector3D force, 
              float masse = 1.0);

    void integrerVerlet(float dt);
};

#endif 
