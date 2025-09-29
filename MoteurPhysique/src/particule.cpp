#include "3DVector.h"
#include "Particule.h"

Particule::Particule(Vector3D pos,
                     Vector3D vel,
                     Vector3D force,
                     float masse)
    : _pos(pos),
      _vel(vel),
      _acc(force)
{
    _oldPos = _pos - _vel.scalar(0.016f); // 60fps
    setMasse(masse);
}

Vector3D Particule::getOldPos() const {
    return _oldPos;
}

Vector3D Particule::getPos() const {
    return _pos;
}

Vector3D Particule::getVx() const {
    return _vel;
}

Vector3D Particule::getForce() const {
    return _acc;
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

void Particule::setAcc(float ax, float ay, float az) {
    _acc = Vector3D(ax, ay, az);
}

void Particule::setAcc(Vector3D accel) {
    _acc = accel;
}

void Particule::setMasse(float masse) {
    if (masse <= 0.0f) {
        _inverseMasse = 0.0f;
    } else {
        _inverseMasse = 1.0f / masse;
    }
}

void Particule::integrerVerlet(Vector3D Force,float dt) {
    if (_inverseMasse <= 0.0f) return;

    const float damping = 0.99f; // proche de 1.0 pour pas tuer la dynamique
    Vector3D acc = Force.scalar(_inverseMasse);

    Vector3D temp = _pos;
    _pos = _pos + (_pos - _oldPos).scalar(damping) + acc.scalar(dt * dt);
    _oldPos = temp;

    _vel = (_pos - _oldPos).scalar(1.f/dt); // vitesse dérivée des positions
}

void Particule::integrerEuler(Vector3D Force, float dt) {
    
	_acc = Force.scalar(_inverseMasse);
	_vel = _vel + _acc.scalar(dt);
	_pos = _pos + _vel.scalar(dt);

}
