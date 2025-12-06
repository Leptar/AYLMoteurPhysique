#pragma once

#include "../MathStruct/3DVector.h"
#include "../MathStruct/Matrix4.h"
#include "../CorpsRigide/CorpsRigide.h"
#include "BoundingSphere.h"

/**
 * @brief Classe de base abstraite pour les primitives de collision.
 * Chaque primitive est associée à un corps rigide et peut calculer sa sphère englobante.
 */
class CollisionPrimitive
{
public:
    CorpsRigide* body;
    Matrix4 offset;  // Transformation locale par rapport au corps rigide

    CollisionPrimitive() : body(nullptr), offset() {}

    /**
     * @brief Calcule la sphère englobante de cette primitive dans l'espace monde.
     */
    virtual BoundingSphere getBoundingSphere() const = 0;

    /**
     * @brief Retourne la transformation monde de cette primitive.
     */
    Matrix4 getTransform() const;

    virtual ~CollisionPrimitive() = default;
};

/**
 * @brief Primitive de collision pour un plan infini.
 * Le plan est défini par une normale et une distance par rapport à l'origine.
 */
class CollisionPlane : public CollisionPrimitive
{
public:
    Vector3D normal;     // Normale du plan (doit être normalisée)
    float offset;        // Distance du plan par rapport à l'origine (le long de la normale)

    CollisionPlane() : normal(0, 1, 0), offset(0) {}

    virtual BoundingSphere getBoundingSphere() const override
    {
        // Un plan infini a une sphère englobante infinie
        return BoundingSphere(Vector3D(), std::numeric_limits<float>::infinity());
    }
};

/**
 * @brief Primitive de collision pour une sphère.
 */
class CollisionSphere : public CollisionPrimitive
{
public:
    float radius;

    CollisionSphere() : radius(0) {}

    virtual BoundingSphere getBoundingSphere() const override
    {
        Vector3D worldPos = body ? body->getPosition() : Vector3D();
        return BoundingSphere(worldPos, radius);
    }
};

/**
 * @brief Primitive de collision pour une boîte alignée aux axes (dans son référentiel local).
 */
class CollisionBox : public CollisionPrimitive
{
public:
    Vector3D halfSize;  // Demi-dimensions de la boîte

    CollisionBox() : halfSize() {}
    CollisionBox(const Vector3D& hs) : halfSize(hs) {}

    virtual BoundingSphere getBoundingSphere() const override
    {
        Vector3D worldPos = body ? body->getPosition() : Vector3D();
        // Le rayon de la sphère englobante est la diagonale de la boîte
        float radius = halfSize.GetNorm();
        return BoundingSphere(worldPos, radius);
    }

    /**
     * @brief Retourne les 8 sommets de la boîte dans l'espace monde.
     */
    void getVertices(Vector3D vertices[8]) const;
};
