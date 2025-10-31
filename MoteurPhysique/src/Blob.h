#pragma once

#include "MathStruct/3DVector.h"
#include "ForceGenerator/ForceFriction.h"
#include "ForceGenerator/ForceGravity.h"
#include "ForceGenerator/ForceRessortParticule.h"
#include "ForceGenerator/ParticuleForceRegistry.h"
#include "SystemeCollision/SystemCollisionDetection.h"
#include "ofColor.h"
#include "ofRectangle.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

class Blob
{
public:
    Blob();

    /// Configure le blob à l'intérieur des limites fournies.
    void setup(const ofRectangle& bounds);
    /// Réinitialise toutes les particules et reconstruit le blob dans ces limites.
    void reset(const ofRectangle& bounds);
    /// Met à jour la zone de jeu sans reconstruire les particules.
    void setBounds(const ofRectangle& bounds);
    /// Intègre la physique et gère les collisions du blob.
    void update(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    /// Dessine les particules du blob ainsi que les ressorts facultatifs.
    void draw(bool showSprings, bool highlightCollisions) const;

    /// Applique les forces issues de l'entrée joueur sur la particule centrale.
    void applyMovement(const Vector3D& inputDirection, float dt);
    /// Marque la prochaine particule périphérique disponible comme détachée.
    bool detachPeripheralParticle();
    /// Rattache toutes les particules précédemment détachées.
    void reattachAllParticles();

    /// Nombre total de particules composant actuellement le blob.
    std::size_t particleCount() const;
    /// Nombre de particules actuellement en collision.
    std::size_t activeCollisionCount() const;
    /// Somme des masses de chaque particule du blob.
    float totalMass() const;


private:
    /// Structure utilitaire qui mémorise quel générateur agit sur quelles particules.
    struct SpringBinding {
        Particule* owner = nullptr;
        Particule* other = nullptr;
        ParticuleForceGenerator* generator = nullptr;
    };

    /// Représentation simplifiée d'un ressort utilisée pour l'affichage.
    struct SpringConnection {
        Particule* a = nullptr;
        Particule* b = nullptr;
        float restLength = 0.f;
    };

    /// Place les particules en anneau autour de la particule centrale contrôlable.
    void buildBlob(const ofRectangle& bounds);
    /// Empile les forces demandées et intègre le mouvement de chaque particule.
    void applyForces(float dt, bool useVerletIntegration, bool applyGravity, bool applyFriction, bool applySprings);
    /// Détecte les recouvrements particule/plan et les résout via le système de collision.
    void detectAndResolveCollisions();

    ofRectangle playArea;
    std::vector<std::unique_ptr<Particule>> particles;
    Particule* centerParticle = nullptr;
    ParticuleForceRegistry registry;
    std::vector<std::unique_ptr<ParticuleForceGenerator>> springGenerators;
    std::vector<SpringBinding> springBindings;
    std::vector<SpringConnection> springConnections;
    std::unordered_set<const Particule*> detachedParticles;

    std::unique_ptr<ForceGravity> gravity;
    std::unique_ptr<ForceFriction> friction;

    SystemCollisionDetection collisionSystem;
    std::unordered_set<const Particule*> collidingParticles;
};
