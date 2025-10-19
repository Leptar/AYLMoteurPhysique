#include "Blob.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace {
constexpr float kGravity = 980.0f;
constexpr float kAirDrag = 4.5f;
constexpr float kBoundaryBounce = 0.35f;
constexpr float kMaxVelocity = 900.0f;
}

Blob::Blob() = default;

void Blob::setup(const Vector3D& center, float radius, std::size_t outerCount) {
    particles.clear();
    links.clear();
    adjacency.clear();
    lastAttachedMask.clear();
    leftHalf.clear();
    rightHalf.clear();
    cachedAttachedCount = 0;
    splitActive = false;
    particleRadius = radius * 0.12f;

    if (outerCount < 4) {
        outerCount = 4;
    }
    if (outerCount % 2 == 1) {
        outerCount += 1;
    }

    particles.reserve(outerCount + 1);
    links.reserve(outerCount * 2);

    // Centre du blob
    particles.emplace_back(std::make_unique<Particule>(center, Vector3D(0, 0, 0), Vector3D(0, 0, 0), 1.5f));
    particles.back()->rayonCollision = particleRadius;
    particles.back()->clearForce();
    rootIndex = 0;

    const float outerRadius = radius;
    const float angleStep = static_cast<float>(TWO_PI / outerCount);

    for (std::size_t i = 0; i < outerCount; ++i) {
        float angle = angleStep * static_cast<float>(i);
        Vector3D pos(center.x + std::cos(angle) * outerRadius,
                     center.y + std::sin(angle) * outerRadius,
                     0.0f);
        particles.emplace_back(std::make_unique<Particule>(pos, Vector3D(0, 0, 0), Vector3D(0, 0, 0), 1.0f));
        particles.back()->rayonCollision = particleRadius;
        particles.back()->clearForce();

        if (i < outerCount / 2) {
            leftHalf.push_back(particles.size() - 1);
        } else {
            rightHalf.push_back(particles.size() - 1);
        }
    }

    const float ringRest = 2.0f * outerRadius * std::sin(angleStep * 0.5f);
    const float ringMax = ringRest * 1.45f;
    const float ringStiffness = 240.0f;
    const float ringCable = ringStiffness * 1.2f;

    const float spokeRest = outerRadius;
    const float spokeMax = spokeRest * 1.35f;
    const float spokeStiffness = 320.0f;
    const float spokeCable = spokeStiffness * 1.3f;

    std::size_t half = outerCount / 2;

    auto addLink = [&](std::size_t a, std::size_t b, float rest, float max, float stiffness, float cable, bool bridge) {
        links.push_back({a, b, rest, max, stiffness, cable, true, bridge});
    };

    for (std::size_t i = 0; i < outerCount; ++i) {
        std::size_t current = 1 + i;
        std::size_t next = 1 + ((i + 1) % outerCount);
        bool bridge = (i == half - 1) || (i == outerCount - 1);
        addLink(current, next, ringRest, ringMax, ringStiffness, ringCable, bridge);
    }

    for (std::size_t i = 0; i < outerCount; ++i) {
        std::size_t outerIndex = 1 + i;
        bool bridge = (i >= half);
        addLink(rootIndex, outerIndex, spokeRest, spokeMax, spokeStiffness, spokeCable, bridge);
    }

    rebuildConnectivity();
    lastAttachedMask = computeAttachedMask();
    cachedAttachedCount = static_cast<std::size_t>(std::count(lastAttachedMask.begin(), lastAttachedMask.end(), true));
}

void Blob::setBounds(const ofRectangle& worldBounds) {
    bounds = worldBounds;
}

void Blob::setObstacles(const std::vector<Obstacle>& shapes) {
    obstacles = shapes;
}

void Blob::setControlAcceleration(const Vector3D& acceleration) {
    controlAcceleration = acceleration;
}

void Blob::resetForces() {
    for (auto& particle : particles) {
        if (!particle) {
            continue;
        }
        particle->clearForce();
        Vector3D gravity(0.0f, kGravity * particle->masse, 0.0f);
        particle->addForce(gravity);
    }
}

void Blob::applyInternalForces(float /*dt*/) {
    for (const auto& link : links) {
        if (!link.active) {
            continue;
        }

        Particule* a = particles[link.a].get();
        Particule* b = particles[link.b].get();
        if (a == nullptr || b == nullptr) {
            continue;
        }

        Vector3D delta = b->_pos - a->_pos;
        float length = delta.GetNorm();
        if (length <= std::numeric_limits<float>::epsilon()) {
            continue;
        }

        Vector3D direction = delta.scalar(1.0f / length);
        float targetLength = link.restLength;
        float stiffness = link.stiffness;

        if (length > link.maxLength) {
            targetLength = link.maxLength;
            stiffness = link.cableStiffness;
        }

        float stretch = length - targetLength;
        Vector3D force = direction.scalar(-stiffness * stretch);

        a->addForce(force);
        b->addForce(force.scalar(-1.0f));
    }
}

void Blob::integrate(float dt) {
    Vector3D dragForce;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        Particule* particle = particles[i].get();
        if (particle == nullptr) {
            continue;
        }

        if (i < lastAttachedMask.size() && lastAttachedMask[i]) {
            particle->addForce(controlAcceleration.scalar(particle->masse));
        }

        dragForce = particle->_vel.scalar(-kAirDrag);
        particle->addForce(dragForce);

        particle->integrerVerlet(dt);

        float speed = particle->_vel.GetNorm();
        if (speed > kMaxVelocity) {
            Vector3D normalized = particle->_vel.normalize();
            Vector3D clamped = normalized.scalar(kMaxVelocity);
            particle->_vel = clamped;
            particle->_oldPos = particle->_pos - particle->_vel.scalar(dt);
        }
    }
}

void Blob::applyBounds(float dt) {
    if (bounds.getWidth() <= 0.0f || bounds.getHeight() <= 0.0f) {
        return;
    }

    for (auto& particlePtr : particles) {
        Particule* particle = particlePtr.get();
        if (particle == nullptr) {
            continue;
        }

        bool collided = false;
        Vector3D velocity = particle->_vel;

        float minX = bounds.getMinX() + particleRadius;
        float maxX = bounds.getMaxX() - particleRadius;
        float minY = bounds.getMinY() + particleRadius;
        float maxY = bounds.getMaxY() - particleRadius;

        if (particle->_pos.x < minX) {
            particle->_pos.x = minX;
            velocity.x *= -kBoundaryBounce;
            collided = true;
        } else if (particle->_pos.x > maxX) {
            particle->_pos.x = maxX;
            velocity.x *= -kBoundaryBounce;
            collided = true;
        }

        if (particle->_pos.y < minY) {
            particle->_pos.y = minY;
            velocity.y *= -kBoundaryBounce;
            collided = true;
        } else if (particle->_pos.y > maxY) {
            particle->_pos.y = maxY;
            velocity.y *= -kBoundaryBounce;
            collided = true;
        }

        if (collided) {
            particle->_vel = velocity;
            particle->_oldPos = particle->_pos - particle->_vel.scalar(dt);
        }
    }
}

void Blob::rebuildConnectivity() {
    adjacency.assign(particles.size(), {});
    for (const auto& link : links) {
        if (!link.active) {
            continue;
        }
        adjacency[link.a].push_back(link.b);
        adjacency[link.b].push_back(link.a);
    }
}

std::vector<bool> Blob::computeAttachedMask() const {
    std::vector<bool> mask(particles.size(), false);
    if (particles.empty()) {
        return mask;
    }

    if (rootIndex >= particles.size()) {
        return mask;
    }

    std::queue<std::size_t> toVisit;
    toVisit.push(rootIndex);
    mask[rootIndex] = true;

    while (!toVisit.empty()) {
        std::size_t current = toVisit.front();
        toVisit.pop();

        if (current >= adjacency.size()) {
            continue;
        }

        for (std::size_t neighbor : adjacency[current]) {
            if (neighbor >= mask.size()) {
                continue;
            }
            if (mask[neighbor]) {
                continue;
            }
            mask[neighbor] = true;
            toVisit.push(neighbor);
        }
    }

    return mask;
}

void Blob::update(float dt) {
    rebuildConnectivity();
    lastAttachedMask = computeAttachedMask();
    cachedAttachedCount = static_cast<std::size_t>(std::count(lastAttachedMask.begin(), lastAttachedMask.end(), true));

    resetForces();
    applyInternalForces(dt);
    integrate(dt);
    applyObstacles(dt);
    applyBounds(dt);

    controlAcceleration = Vector3D(0, 0, 0);
}

void Blob::draw() const {
    ofPushStyle();

    ofSetLineWidth(3.0f);
    ofSetColor(90, 120, 255, 120);
    for (const auto& link : links) {
        if (!link.active) {
            continue;
        }
        const Particule* a = particles[link.a].get();
        const Particule* b = particles[link.b].get();
        if (a == nullptr || b == nullptr) {
            continue;
        }
        ofDrawLine(a->_pos.x, a->_pos.y, b->_pos.x, b->_pos.y);
    }

    for (std::size_t i = 0; i < particles.size(); ++i) {
        const Particule* particle = particles[i].get();
        if (particle == nullptr) {
            continue;
        }

        bool attached = (i < lastAttachedMask.size()) ? lastAttachedMask[i] : false;
        if (attached) {
            ofSetColor(255, 180, 0, 220);
        } else {
            ofSetColor(120, 200, 255, 200);
        }

        ofDrawCircle(particle->_pos.x, particle->_pos.y, particleRadius);
    }

    ofPopStyle();
}

void Blob::applyObstacles(float dt) {
    if (obstacles.empty()) {
        return;
    }

    const float bounce = kBoundaryBounce;

    for (auto& particlePtr : particles) {
        Particule* particle = particlePtr.get();
        if (particle == nullptr) {
            continue;
        }

        Vector3D velocity = particle->_vel;
        bool collided = false;

        for (const Obstacle& obstacle : obstacles) {
            if (obstacle.type == Obstacle::Type::Rectangle) {
                float expandedLeft = obstacle.rect.getMinX() - particleRadius;
                float expandedRight = obstacle.rect.getMaxX() + particleRadius;
                float expandedTop = obstacle.rect.getMinY() - particleRadius;
                float expandedBottom = obstacle.rect.getMaxY() + particleRadius;

                if (particle->_pos.x <= expandedLeft || particle->_pos.x >= expandedRight ||
                    particle->_pos.y <= expandedTop || particle->_pos.y >= expandedBottom) {
                    continue;
                }

                float leftPenetration = particle->_pos.x - expandedLeft;
                float rightPenetration = expandedRight - particle->_pos.x;
                float topPenetration = particle->_pos.y - expandedTop;
                float bottomPenetration = expandedBottom - particle->_pos.y;

                float minPenetration = std::min(std::min(leftPenetration, rightPenetration),
                                                 std::min(topPenetration, bottomPenetration));

                if (minPenetration == leftPenetration) {
                    particle->_pos.x = expandedLeft;
                    velocity.x *= -bounce;
                } else if (minPenetration == rightPenetration) {
                    particle->_pos.x = expandedRight;
                    velocity.x *= -bounce;
                } else if (minPenetration == topPenetration) {
                    particle->_pos.y = expandedTop;
                    velocity.y *= -bounce;
                } else {
                    particle->_pos.y = expandedBottom;
                    velocity.y *= -bounce;
                }

                collided = true;
            } else {
                Vector3D center = obstacle.center;
                Vector3D offset(particle->_pos.x - center.x, particle->_pos.y - center.y, 0.0f);
                float distance = offset.GetNorm();
                float allowed = obstacle.radius + particleRadius;

                if (distance >= allowed || allowed <= std::numeric_limits<float>::epsilon()) {
                    continue;
                }

                Vector3D normal;
                if (distance <= std::numeric_limits<float>::epsilon()) {
                    normal = Vector3D(1.0f, 0.0f, 0.0f);
                } else {
                    normal = offset.scalar(1.0f / distance);
                }

                particle->_pos.x = center.x + normal.x * allowed;
                particle->_pos.y = center.y + normal.y * allowed;

                float vn = velocity.dot(normal);
                if (vn < 0.0f) {
                    velocity = velocity - normal.scalar((1.0f + bounce) * vn);
                }

                collided = true;
            }
        }

        if (collided) {
            particle->_vel = velocity;
            particle->_oldPos = particle->_pos - particle->_vel.scalar(dt);
        }
    }
}

void Blob::split() {
    if (splitActive) {
        return;
    }

    for (auto& link : links) {
        if (link.bridge) {
            link.active = false;
        }
    }

    const float dt = 1.0f / 60.0f;
    const float impulse = 140.0f;
    for (std::size_t idx : leftHalf) {
        if (idx >= particles.size()) {
            continue;
        }
        Particule* particle = particles[idx].get();
        if (particle == nullptr) {
            continue;
        }
        particle->_vel.x -= impulse;
        Vector3D delta = particle->_vel.scalar(dt);
        particle->_oldPos = particle->_pos - delta;
    }
    for (std::size_t idx : rightHalf) {
        if (idx >= particles.size()) {
            continue;
        }
        Particule* particle = particles[idx].get();
        if (particle == nullptr) {
            continue;
        }
        particle->_vel.x += impulse;
        Vector3D delta = particle->_vel.scalar(dt);
        particle->_oldPos = particle->_pos - delta;
    }

    splitActive = true;
    rebuildConnectivity();
}

void Blob::merge() {
    if (!splitActive) {
        return;
    }

    for (auto& link : links) {
        link.active = true;
    }
    const float dt = 1.0f / 60.0f;
    for (auto& particlePtr : particles) {
        Particule* particle = particlePtr.get();
        if (particle == nullptr) {
            continue;
        }
        particle->_vel = particle->_vel.scalar(0.5f);
        Vector3D delta = particle->_vel.scalar(dt);
        particle->_oldPos = particle->_pos - delta;
    }
    splitActive = false;
    rebuildConnectivity();
}
