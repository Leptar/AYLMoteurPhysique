#pragma once

#include <cstddef>
#include <vector>

#include "Blob.h"

class World {
public:
    void setup(const ofRectangle& bounds);
    void update(float dt);
    void draw() const;

    void setControlAcceleration(const Vector3D& acceleration);
    void splitBlob();
    void mergeBlob();

    void setObstacles(const std::vector<ofRectangle>& cubes);

    std::size_t getAttachedCount() const;

private:
    Blob blob;
    Vector3D controlAcceleration;
    ofRectangle worldBounds;
    std::vector<ofRectangle> obstacles;
};
