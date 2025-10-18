#include "3DVector.h"
#include "Particule.h"

Particule::Particule(Vector3D pos,
                     Vector3D vel,
                     Vector3D force,
                     float masse,
                     float linear,
                     float quadratic,
                     float rayonCollision)
    : _pos(pos),
      _vel(vel),
      _acc(force),
	  linearFriction(linear),
	  quadraticFriction(quadratic),
      rayonCollision(rayonCollision)
{
    _oldPos = _pos - _vel.scalar(0.016f); // 60fps
    if (masse <= 0.0f) {
        _inverseMasse = 0.0f;
    } else {
        _inverseMasse = 1.0f / masse;
    }
}

void Particule::integrerVerlet(float dt) {
    if (_inverseMasse <= 0.0f) return;

    const float damping = 0.99f; // proche de 1.0 pour pas tuer la dynamique
    Vector3D acc = AccumForce.scalar(_inverseMasse);

    Vector3D temp = _pos;
    _pos = _pos + (_pos - _oldPos).scalar(damping) + acc.scalar(dt * dt);
    _oldPos = temp;

    _vel = (_pos - _oldPos).scalar(1.f/dt); // vitesse dérivée des positions
	clearForce();
}

void Particule::addForce(const Vector3D & Force) {
	AccumForce = AccumForce + Force;
}

void Particule::clearForce() {
	AccumForce = Vector3D(0, 0, 0);
}

void Particule::integrerEuler(float dt) {

	_acc = AccumForce.scalar(_inverseMasse);
	_vel = _vel + _acc.scalar(dt);
	_pos = _pos + _vel.scalar(dt);

	clearForce();
}
