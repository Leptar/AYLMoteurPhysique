//
// Created by leoul on 2025-09-09.
//

#ifndef MOTEURPHYSIQUE_VECTOR3D_H
#define MOTEURPHYSIQUE_VECTOR3D_H


class Vector3D {
public:
    Vector3D();
    Vector3D(float x, float y, float z);

    // produit par un scalaire
    Vector3D operator*(const float &v) const;

    Vector3D operator+(const Vector3D &v) const;
    Vector3D operator-(const Vector3D &v) const;

    Vector3D normalize();
    float dot(const Vector3D &v) const;
    Vector3D cross(const Vector3D &v) const;

    float GetNorm();
    float GetSquareNorm();

public:
    float x, y, z;
};


#endif //MOTEURPHYSIQUE_VECTOR3D_H
