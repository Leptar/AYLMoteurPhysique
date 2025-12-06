#pragma once
#include "3DVector.h"
#include "Primitive.h"

class Plane : public Primitive
{
public:
    Vector3D normal;
    float PlaneOffset;
};
