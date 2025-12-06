#pragma once
#include "3DVector.h"

struct AABB
{
    Vector3D min;
    Vector3D max;

    AABB() = default;
    AABB(const Vector3D& minPoint, const Vector3D& maxPoint) : min(minPoint), max(maxPoint) {}

    Vector3D getCenter() const
    {
        return (min + max).scalar(0.5f);
    }

    // Vérifie si une autre AABB est entièrement contenue dans celle-ci
    bool contains(const AABB& other) const
    {
        return other.min.x >= min.x && other.max.x <= max.x &&
               other.min.y >= min.y && other.max.y <= max.y &&
               other.min.z >= min.z && other.max.z <= max.z;
    }

    // Vérifie si cette AABB intersecte une autre
    bool intersects(const AABB& other) const {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y &&
               min.z <= other.max.z && max.z >= other.min.z;
    }
};
