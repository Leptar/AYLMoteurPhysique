#pragma once

#include <ostream>

// ---------------------------------------------------------------------------
// Classe utilitaire représentant un vecteur 3D. Toutes les opérations sont
// écrites de manière sûre afin d'éviter les divisions inutiles ou les copies
// temporaires. Les commentaires sont en français pour faciliter la prise en
// main par l'équipe.
// ---------------------------------------------------------------------------
class Vector3D {
public:
    float x;
    float y;
    float z;

    Vector3D() = default;
    Vector3D(float xValue, float yValue, float zValue);

    // Retourne un vecteur nul.
    static Vector3D zero();

    // Calcule la distance euclidienne entre deux points.
    static float distance(const Vector3D& a, const Vector3D& b);

    [[nodiscard]] float norme() const;
    [[nodiscard]] float normeCarree() const;

    [[nodiscard]] Vector3D normalise() const;
    [[nodiscard]] float produitScalaire(const Vector3D& autre) const;
    [[nodiscard]] Vector3D produitVectoriel(const Vector3D& autre) const;

    [[nodiscard]] Vector3D operator+(const Vector3D& autre) const;
    [[nodiscard]] Vector3D operator-(const Vector3D& autre) const;
    [[nodiscard]] Vector3D operator*(float scalaire) const;
    [[nodiscard]] Vector3D operator/(float scalaire) const;

    Vector3D& operator+=(const Vector3D& autre);
    Vector3D& operator-=(const Vector3D& autre);
    Vector3D& operator*=(float scalaire);
    Vector3D& operator/=(float scalaire);
};

// Permet d'afficher le vecteur dans les logs de debug.
std::ostream& operator<<(std::ostream& os, const Vector3D& v);

