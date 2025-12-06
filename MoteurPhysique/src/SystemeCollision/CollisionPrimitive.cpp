#include "CollisionPrimitive.h"

Matrix4 CollisionPrimitive::getTransform() const
{
    if (!body) {
        return offset;
    }

    // Combiner la transformation du corps avec l'offset local
    Matrix4 bodyTransform = body->getTransformMatrix();
    return bodyTransform * offset;
}

void CollisionBox::getVertices(Vector3D vertices[8]) const
{
    if (!body) {
        return;
    }

    Matrix4 transform = getTransform();

    // Les 8 sommets dans l'espace local
    Vector3D localVertices[8] = {
        Vector3D(-halfSize.x, -halfSize.y, -halfSize.z),
        Vector3D( halfSize.x, -halfSize.y, -halfSize.z),
        Vector3D(-halfSize.x,  halfSize.y, -halfSize.z),
        Vector3D( halfSize.x,  halfSize.y, -halfSize.z),
        Vector3D(-halfSize.x, -halfSize.y,  halfSize.z),
        Vector3D( halfSize.x, -halfSize.y,  halfSize.z),
        Vector3D(-halfSize.x,  halfSize.y,  halfSize.z),
        Vector3D( halfSize.x,  halfSize.y,  halfSize.z)
    };

    // Transformer les sommets dans l'espace monde
    for (int i = 0; i < 8; i++) {
        vertices[i] = transform.transform(localVertices[i]);
    }
}
