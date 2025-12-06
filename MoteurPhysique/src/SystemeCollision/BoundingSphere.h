#pragma once

#include "../MathStruct/3DVector.h"

/**
 * @brief Représente une sphère englobante utilisée pour la phase élargie de détection de collision.
 */
struct BoundingSphere
{
    Vector3D center;
    float radius;

    BoundingSphere() : center(), radius(0.0f) {}
    BoundingSphere(const Vector3D& c, float r) : center(c), radius(r) {}

    /**
     * @brief Teste si cette sphère chevauche une autre sphère.
     */
    bool overlaps(const BoundingSphere& other) const
    {
        float distanceSquared = (center - other.center).GetNormSquared();
        float radiusSum = radius + other.radius;
        return distanceSquared < (radiusSum * radiusSum);
    }

    /**
     * @brief Calcule le volume de cette sphère.
     */
    float getVolume() const
    {
        return (4.0f / 3.0f) * 3.14159265f * radius * radius * radius;
    }

    /**
     * @brief Calcule une sphère englobante qui contient deux sphères.
     */
    static BoundingSphere combine(const BoundingSphere& a, const BoundingSphere& b)
    {
        Vector3D centerOffset = b.center - a.center;
        float distance = centerOffset.GetNorm();

        // Si une sphère contient l'autre, retourner la plus grande
        if (distance + b.radius <= a.radius) {
            return a;
        }
        if (distance + a.radius <= b.radius) {
            return b;
        }

        // Calculer la nouvelle sphère
        float newRadius = (distance + a.radius + b.radius) * 0.5f;
        Vector3D newCenter = a.center;
        if (distance > 0) {
            newCenter = newCenter + centerOffset.scalar((newRadius - a.radius) / distance);
        }

        return BoundingSphere(newCenter, newRadius);
    }
};
