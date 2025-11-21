#pragma once    
#include "CorpsRigide.h"

struct Primitive
{
    // Lien entre physique et collision
    CorpsRigide* corpsRigide;
    
    // Transforme les coordonnées locales en coordonnées globales
    Matrix4 offset;
    
    Matrix4 GetTransformMatrix() const;

    Primitive();
    virtual ~Primitive() = default;
};
