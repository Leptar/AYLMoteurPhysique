#include "Primitive.h"
Matrix4 Primitive::GetTransformMatrix() const
{
    if (!corpsRigide)
    {
        return offset;
    }
    return corpsRigide->getTransformMatrix() * offset;
}

Primitive::Primitive():
    corpsRigide(nullptr),
    offset(Matrix4::Identity())
{}
