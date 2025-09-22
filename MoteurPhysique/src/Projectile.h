#pragma once
#include "3DVector.h"
#include "ofColor.h"
#include "particule.h"

class ofVec3f;

/// Types de projectiles disponibles dans la simulation.
enum class ProjectileType { Balle, Boulet, Laser, BouleDeFeu };

/// Paramètres d'initialisation pour un projectile.
struct ProjectileConfig {
    /// Masse utilisée pour calculer l'inertie et la gravité du projectile.
    float masse;
    /// Vitesse initiale exprimée dans le repère écran.
    Vector3D vitesseInitiale;
    /// Couleur de rendu associée à ce projectile.
    ofColor couleur;
};

/// Représente un projectile affiché à l'écran et animé dans le temps.
class Projectile
{
public:
    /// Particule physique sous-jacente animant le projectile.
    Particule* particule;
    /// Type logique du projectile (utilisé pour les interactions utilisateur).
    ProjectileType type;
    /// Couleur utilisée pour le rendu.
    ofColor couleur;
    /// Points échantillonnés de la trajectoire pour le rendu.
    std::vector<ofVec3f> trajectoire;

    /// Construit un projectile à partir de sa configuration et d'une position initiale.
    Projectile(ProjectileType T, const ProjectileConfig& config, const Vector3D& position);

    /// Met à jour la dynamique du projectile pour un pas de temps donné.
    void update(float deltaTime);

    /// Dessine le projectile et sa trajectoire.
    void draw() const;
};
