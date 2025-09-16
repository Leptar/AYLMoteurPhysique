#ifndef PARTICULE_H
#define PARTICULE_H

#include "3DVector.h"
#include <cmath>

class Particule {
private:
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

    Vector3D getOldPos() const;

    Vector3D getPos() const;

    Vector3D getVx() const;

    Vector3D getForce() const;

    float getInverseMasse() const;

    void setOldPosition(float px, float py, float pz);
    void setOldPosition(Vector3D pos);

    void setPosition(float px, float py, float pz);
    void setPosition(Vector3D pos);

    void setVitesse(float vx, float vy, float vz);
    void setVitesse(Vector3D vel);

    void setForce(float ax, float ay, float az);
    void setForce(Vector3D force);

    void setMasse(float masse);

    void integrerVerlet(float dt);
};

#endif 
