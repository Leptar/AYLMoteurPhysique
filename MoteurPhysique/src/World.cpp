#include "World.h"

#include "ofGraphics.h"
#include "ofMain.h"

void World::setup(const ofRectangle& bounds) {
    worldBounds = bounds;
    blob.setup(Vector3D(bounds.getCenter().x, bounds.getCenter().y, 0.0f), bounds.getWidth() * 0.055f, 12);
    blob.setBounds(worldBounds);
    blob.setObstacles(obstacles);
    controlAcceleration = Vector3D(0, 0, 0);
}

void World::update(float dt) {
    blob.setBounds(worldBounds);
    blob.setObstacles(obstacles);
    blob.setControlAcceleration(controlAcceleration);
    blob.update(dt);
    controlAcceleration = Vector3D(0, 0, 0);
}

void World::draw() const {
    ofPushStyle();
    ofSetColor(80, 120, 200, 180);
    for (const Blob::Obstacle& obstacle : obstacles) {
        switch (obstacle.type) {
        case Blob::Obstacle::Type::Rectangle:
            ofDrawRectangle(obstacle.rect);
            break;
        case Blob::Obstacle::Type::Circle:
            ofDrawCircle(obstacle.center.x, obstacle.center.y, obstacle.radius);
            break;
        }
    }
    ofPopStyle();

    blob.draw();
}

void World::setControlAcceleration(const Vector3D& acceleration) {
    controlAcceleration = acceleration;
}

void World::setObstacles(const std::vector<Blob::Obstacle>& shapes) {
    obstacles = shapes;
    blob.setObstacles(obstacles);
}

void World::splitBlob() {
    blob.split();
}

void World::mergeBlob() {
    blob.merge();
}

std::size_t World::getAttachedCount() const {
    return blob.attachedCount();
}
