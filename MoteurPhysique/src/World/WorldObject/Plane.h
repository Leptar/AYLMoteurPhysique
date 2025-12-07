#pragma once
#include "3DVector.h"
#include "Primitive.h"

class Plane : public Primitive
{
public:
    Plane() {
        // Un plan est statique, il n'a pas de corps rigide associé.
        corpsRigide = nullptr;
    }

    Vector3D normal;
    float PlaneOffset;
};
