#include <utf8/unchecked.h>
#include "3DVectorTest.h"
#include "3DVector.h"
#include "Particule.h"

Particule::Particule(Vector3D pos,
                     Vector3D vel,
                     Vector3D force, 
                     float masse)
    : _pos(pos), 
      _vel(vel), 
      _force(force) 
{
    _oldPos = _pos - _vel.scalar(0.016f); // 60fps
    setMasse(masse);
}

/*void Particule::integrerVerlet(float dt) {
    if (_inverseMasse <= 0.0f) return; 

    const float damping = 0.7f; 
    Vector3D acc = _force.scalar(_inverseMasse);
    Vector3D newPos = _pos.scalar(2.f) - _oldPos + acc.scalar(std::pow(dt,2));

    _oldPos = _pos;
    _pos = newPos;
    _vel = _vel.scalar(damping) + acc.scalar(dt);  
}*/

void Particule::integrerVerlet(float dt) {
    if (_inverseMasse <= 0.0f) return;

    const float damping = 0.99f; // proche de 1.0 pour pas tuer la dynamique
    Vector3D acc = _force.scalar(_inverseMasse);

    Vector3D temp = _pos;
    _pos = _pos + (_pos - _oldPos).scalar(damping) + acc.scalar(dt * dt);
    _oldPos = temp;

    _vel = (_pos - _oldPos).scalar(1.f/dt); // vitesse dérivée des positions
}