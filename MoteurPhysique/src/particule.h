#pragma once

#include "3DVector.h"

// ---------------------------------------------------------------------------
// Représente une particule soumise à différentes forces. La classe expose des
// intégrateurs de type Euler semi-explicite et Verlet positionnel afin de
// couvrir les différents besoins du moteur. Les attributs sont publics pour
// faciliter l'interfaçage avec openFrameworks, mais l'API garantit la cohérence
// via des méthodes utilitaires.
// ---------------------------------------------------------------------------
class Particule {
public:
    Vector3D _oldPos{};
    Vector3D _pos{};
    Vector3D _vel{};
    Vector3D _acc{};
    Vector3D accumulatedForce{};

    float masse = 1.0f;
    float _inverseMasse = 1.0f;
    float linearFriction = 0.0f;
    float quadraticFriction = 0.0f;
    float rayonCollision = 0.0f;

    Particule(const Vector3D& position,
              const Vector3D& vitesse,
              const Vector3D& acceleration,
              float masseValeur = 1.0f,
              float frictionLineaire = 0.0f,
              float frictionQuadratique = 0.0f,
              float rayon = 0.0f);

    [[nodiscard]] bool estFixe() const;

    void integrerEuler(float dt);
    void integrerVerlet(float dt);

    void addForce(const Vector3D& force);
    void clearForce();
};

