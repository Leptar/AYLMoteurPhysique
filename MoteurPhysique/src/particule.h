#ifndef PARTICULE_H
#define PARTICULE_H

#include "3DVector.h"

class Particule {
public:
    Vector3D _oldPos;

    Vector3D _pos;

    Vector3D _vel;

    Vector3D _acc;

    float _inverseMasse;
    
    Particule(Vector3D pos,
              Vector3D vel,
              Vector3D acc,
              float masse = 1.0);

    void integrerEuler(Vector3D, float dt);
    void integrerVerlet(Vector3D Force, float dt);

};

#endif
