//
// Created by leoul on 2025-09-09.
//

#include "3DVector.h"

#include <cmath>

/// Initialise un vecteur nul.
Vector3D::Vector3D() {
    x = 0;
    y = 0;
    z = 0;
}

/// Initialise un vecteur avec les composantes indiquées.
Vector3D::Vector3D(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

/// Multiplie les composantes par un scalaire et retourne le résultat.
Vector3D Vector3D::scalar(const float &v) const {
    return Vector3D(x * v, y * v, z * v);
}

/// Retourne la distance euclidienne entre deux vecteurs.
float Vector3D::distance(Vector3D& v, Vector3D& t)
{
    Vector3D Vdist = v - t;
    return Vdist.GetNorm();
}

/// Additionne deux vecteurs.
Vector3D Vector3D::operator+(const Vector3D &v) const {
    return Vector3D(x + v.x, y + v.y, z + v.z);
}

/// Soustrait deux vecteurs.
Vector3D Vector3D::operator-(const Vector3D &v) const {
    return Vector3D(x - v.x, y - v.y, z - v.z);
}

/// Normalise le vecteur en place.
Vector3D Vector3D::normalize() {
    float norm = GetNorm();
    x = x / norm;
    y = y / norm;
    z = z / norm;
    return *this;
}

/// Calcule le produit scalaire avec un autre vecteur.
float Vector3D::dot(const Vector3D &v) const {
    return x * v.x + y * v.y + z * v.z;
}

/// Calcule le produit vectoriel avec un autre vecteur.
Vector3D Vector3D::cross(const Vector3D &v) const {
    float x1 = y * v.z - z * v.y;
    float y1 = z * v.x - x * v.z;
    float z = x * v.y - y * v.x;

    return Vector3D(x1, y1, z);
}

/// Retourne la norme euclidienne.
float Vector3D::GetNorm() {
    return sqrtf(x * x + y * y + z * z);
}

/// Retourne la norme au carré.
float Vector3D::GetSquareNorm() {
    return x * x + y * y + z * z;
}
