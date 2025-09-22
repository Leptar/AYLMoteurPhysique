#ifndef PARTICULE_H
#define PARTICULE_H

#include "3DVector.h"
#include <cmath>

/// Particule simulée par intégration de Verlet.
class Particule {
    /// Position de l'image précédente, utilisée par l'intégration de Verlet.
    Vector3D _oldPos;

    /// Position courante de la particule.
    Vector3D _pos;

    /// Vitesse instantanée de la particule.
    Vector3D _vel;

    /// Force appliquée durant le pas de temps courant.
    Vector3D _force;

    /// Inverse de la masse (0 pour une particule immobile).
    float _inverseMasse;

public:
    /// Crée une particule en précisant position, vitesse, force initiale et masse.
    Particule(Vector3D pos,
              Vector3D vel,
              Vector3D force,
              float masse = 1.0);

    /// Retourne l'ancienne position utilisée par l'intégrateur.
    Vector3D getOldPos() const;

    /// Retourne la position actuelle.
    Vector3D getPos() const;

    /// Retourne la vitesse actuelle.
    Vector3D getVx() const;

    /// Retourne la force appliquée.
    Vector3D getForce() const;

    /// Retourne l'inverse de la masse (0 si masse infinie).
    float getInverseMasse() const;

    /// Définit l'ancienne position à partir de composantes scalaires.
    void setOldPosition(float px, float py, float pz);
    /// Définit l'ancienne position avec un vecteur complet.
    void setOldPosition(Vector3D pos);

    /// Définit la position courante à partir de composantes scalaires.
    void setPosition(float px, float py, float pz);
    /// Définit la position courante avec un vecteur complet.
    void setPosition(Vector3D pos);

    /// Définit la vitesse à partir de composantes scalaires.
    void setVitesse(float vx, float vy, float vz);
    /// Définit la vitesse via un vecteur.
    void setVitesse(Vector3D vel);

    /// Définit la force à partir de composantes scalaires.
    void setForce(float ax, float ay, float az);
    /// Définit la force via un vecteur.
    void setForce(Vector3D force);

    /// Met à jour l'inverse de la masse.
    void setMasse(float masse);

    /// Intègre la particule par Verlet pour le pas de temps fourni.
    void integrerVerlet(float dt);
};

#endif
