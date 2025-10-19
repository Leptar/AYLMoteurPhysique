//
// Created by leoul on 2025-09-09.
//

#include "3DVector.h"

#include <cmath>
#include <limits>

namespace {
constexpr float kEpsilon = 1e-6f;
}

Vector3D::Vector3D(float xValue, float yValue, float zValue)
    : x(xValue), y(yValue), z(zValue) {}

Vector3D Vector3D::zero() { return Vector3D(0.0f, 0.0f, 0.0f); }

float Vector3D::distance(const Vector3D& a, const Vector3D& b) {
    return (a - b).norme();
}

float Vector3D::norme() const {
    return std::sqrt(normeCarree());
}

float Vector3D::normeCarree() const {
    return x * x + y * y + z * z;
}

Vector3D Vector3D::normalise() const {
    const float magnitude = norme();
    if (magnitude <= kEpsilon) {
        return Vector3D::zero();
    }
    return *this / magnitude;
}

float Vector3D::produitScalaire(const Vector3D& autre) const {
    return x * autre.x + y * autre.y + z * autre.z;
}

Vector3D Vector3D::produitVectoriel(const Vector3D& autre) const {
    return Vector3D(
        y * autre.z - z * autre.y,
        z * autre.x - x * autre.z,
        x * autre.y - y * autre.x);
}

Vector3D Vector3D::operator+(const Vector3D& autre) const {
    return Vector3D(x + autre.x, y + autre.y, z + autre.z);
}

Vector3D Vector3D::operator-(const Vector3D& autre) const {
    return Vector3D(x - autre.x, y - autre.y, z - autre.z);
}

Vector3D Vector3D::operator*(float scalaire) const {
    return Vector3D(x * scalaire, y * scalaire, z * scalaire);
}

Vector3D Vector3D::operator/(float scalaire) const {
    if (std::fabs(scalaire) <= kEpsilon) {
        return Vector3D::zero();
    }
    return Vector3D(x / scalaire, y / scalaire, z / scalaire);
}

Vector3D& Vector3D::operator+=(const Vector3D& autre) {
    x += autre.x;
    y += autre.y;
    z += autre.z;
    return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& autre) {
    x -= autre.x;
    y -= autre.y;
    z -= autre.z;
    return *this;
}

Vector3D& Vector3D::operator*=(float scalaire) {
    x *= scalaire;
    y *= scalaire;
    z *= scalaire;
    return *this;
}

Vector3D& Vector3D::operator/=(float scalaire) {
    if (std::fabs(scalaire) <= kEpsilon) {
        x = y = z = 0.0f;
    } else {
        x /= scalaire;
        y /= scalaire;
        z /= scalaire;
    }
    return *this;
}

std::ostream& operator<<(std::ostream& os, const Vector3D& v) {
    os << "Vector3D(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

