#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "3DVector.h"
#include "particule.h"
#include "ofMain.h"

class Blob {
public:
    Blob();

    void setup(const Vector3D& center, float radius, std::size_t outerCount);
    void setBounds(const ofRectangle& worldBounds);
    void setObstacles(const std::vector<ofRectangle>& cubes);
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
    std::vector<ofRectangle> obstacles;

    float particleRadius = 12.0f;
    bool splitActive = false;
    std::size_t rootIndex = 0;
};
