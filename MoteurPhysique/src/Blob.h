#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "3DVector.h"
#include "Particule.h"
#include "ofMain.h"

// ---------------------------------------------------------------------------
// Blob souple composé de particules reliées par des ressorts. Il peut être
// séparé en deux moitiés et interagit avec un ensemble d'obstacles.
// ---------------------------------------------------------------------------
class Blob {
public:
    Blob();

    struct Obstacle {
        enum class Type { Rectangle, Circle };

        // Fabriques utilitaires pour simplifier la construction côté application.
        static Obstacle creerRectangle(const ofRectangle& forme);
        static Obstacle creerCercle(const Vector3D& centre, float rayon);

        Type type = Type::Rectangle;
        ofRectangle rect;
        Vector3D center;
        float radius = 0.0f;
    };

    void setup(const Vector3D& center, float radius, std::size_t outerCount);
    void setBounds(const ofRectangle& worldBounds);
    void setObstacles(const std::vector<Obstacle>& shapes);
    void setControlAcceleration(const Vector3D& acceleration);

    void update(float dt);
    void draw() const;

    void split();
    void merge();

    std::size_t attachedCount() const { return cachedAttachedCount; }

private:
    struct Link {
        std::size_t a;
        std::size_t b;
        float restLength;
        float maxLength;
        float stiffness;
        float cableStiffness;
        bool active;
        bool bridge;
    };

    void resetForces();
    void applyInternalForces(float dt);
    void integrate(float dt);
    void applyBounds(float dt);
    void applyObstacles(float dt);
    void rebuildConnectivity();

    std::vector<bool> computeAttachedMask() const;

    std::vector<std::unique_ptr<Particule>> particles;
    std::vector<Link> links;
    std::vector<std::vector<std::size_t>> adjacency;
    std::vector<std::size_t> leftHalf;
    std::vector<std::size_t> rightHalf;

    std::vector<bool> lastAttachedMask;
    std::size_t cachedAttachedCount = 0;

    Vector3D controlAcceleration;
    ofRectangle bounds;
    std::vector<Obstacle> obstacles;

    float particleRadius = 12.0f;
    bool splitActive = false;
    std::size_t rootIndex = 0;
};

