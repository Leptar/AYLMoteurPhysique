#include "Particule.h"

#include <algorithm>

namespace {
constexpr float kDefaultDt = 1.0f / 60.0f;
}

Particule::Particule(const Vector3D& position,
                     const Vector3D& vitesse,
                     const Vector3D& acceleration,
                     float masseValeur,
                     float frictionLineaire,
                     float frictionQuadratique,
                     float rayon)
    : _oldPos(position - vitesse * kDefaultDt),
      _pos(position),
      _vel(vitesse),
      _acc(acceleration),
      accumulatedForce(Vector3D::zero()),
      masse(std::max(masseValeur, 0.0f)),
      linearFriction(std::max(frictionLineaire, 0.0f)),
      quadraticFriction(std::max(frictionQuadratique, 0.0f)),
      rayonCollision(std::max(rayon, 0.0f))
{
    _inverseMasse = (masse <= 0.0f) ? 0.0f : 1.0f / masse;
}

bool Particule::estFixe() const {
    return _inverseMasse <= 0.0f;
}

void Particule::integrerVerlet(float dt) {
    if (estFixe()) {
        clearForce();
        return;
    }

    const float amortissement = 0.99f;
    const Vector3D accelerationTotale = accumulatedForce * _inverseMasse;

    const Vector3D positionCourante = _pos;
    _pos = _pos + (_pos - _oldPos) * amortissement + accelerationTotale * (dt * dt);
    _oldPos = positionCourante;

    _vel = (_pos - _oldPos) / std::max(dt, 1e-6f);
    clearForce();
}

void Particule::addForce(const Vector3D& force) {
    accumulatedForce += force;
}

void Particule::clearForce() {
    accumulatedForce = Vector3D::zero();
}

void Particule::integrerEuler(float dt) {
    if (estFixe()) {
        clearForce();
        return;
    }

    _acc = accumulatedForce * _inverseMasse;
    _vel += _acc * dt;
    _pos += _vel * dt;

    clearForce();
}

