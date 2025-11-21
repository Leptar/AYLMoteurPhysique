#include "Primitive.h"
Matrix4 Primitive::GetTransformMatrix() const
{
    return corpsRigide->getTransformMatrix() * offset;
}

Primitive::Primitive():
    corpsRigide(nullptr),
    offset(Matrix4::Identity())
{}
