#pragma once
#include "3DVector.h"
#include "Primitive.h"

class Box : public Primitive
{
public:
	Box(Vector3D HalfExtent) : HalfExtent(HalfExtent) {}
    Vector3D HalfExtent;
};
