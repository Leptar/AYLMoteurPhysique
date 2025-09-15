#include "Particule.h"

Particule::Particule(Vector3D oldPos,
                     Vector3D pos,
                     Vector3D vel,
                     Vector3D force, 
                     float masse)
    : _pos(pos), 
      _vel(vel), 
      _force(force) 
{
    setMasse(masse);
}

Vector3D Particule::getPos() const {
    return _pos;
}

Vector3D Particule::getVx() const {
    return _vel;
}

Vector3D Particule::getForce() const {
    return _force;
}

float Particule::getInverseMasse() const {
    return _inverseMasse;
}

void Particule::setOldPosition(float px, float py, float pz) {
    _oldPos = Vector3D(px, py, pz);
}

void Particule::setOldPosition(Vector3D oldPos) {
    _oldPos = oldPos;
}

void Particule::setPosition(float px, float py, float pz) {
    _pos = Vector3D(px, py, pz);
}

void Particule::setPosition(Vector3D pos) {
    _pos = pos;
}

void Particule::setVitesse(float vx, float vy, float vz) {
    _vel = Vector3D(vx, vy, vz);
}

void Particule::setVitesse(Vector3D vel) {
    _vel = vel;
}

void Particule::setForce(float ax, float ay, float az) {
    _force = Vector3D(ax, ay, az);
}

void Particule::setForce(Vector3D force) {
    _force = force;
}

void Particule::setMasse(float masse) {
    if (masse <= 0.0f) {
        _inverseMasse = 0.0;
    } else {
        _inverseMasse = 1.0 / masse;
    }
}

void Particule::integrerVerlet(float dt) {
    if (_inverseMasse <= 0.0) return; 

    const float damping = 0.7; 

    Vector3D acc = _force * _inverseMasse;
    Vector3D newPos = _pos * 2.f - _oldPos + acc * std::pow(dt,2);

    _oldPos = _pos;
    _pos = newPos;

    _vel = _vel * damping + acc * dt;  
}