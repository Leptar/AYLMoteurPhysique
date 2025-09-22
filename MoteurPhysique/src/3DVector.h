//
// Created by leoul on 2025-09-09.
//

#ifndef MOTEURPHYSIQUE_VECTOR3D_H
#define MOTEURPHYSIQUE_VECTOR3D_H

/// Représente un vecteur tridimensionnel avec les opérations usuelles.
class Vector3D {
public:
    /// Construit un vecteur nul.
    Vector3D();
    /// Construit un vecteur initialisé avec les composantes fournies.
    Vector3D(float x, float y, float z);

    /// Retourne ce vecteur multiplié par un scalaire.
    Vector3D scalar(const float &v) const;

    /// Calcule la distance euclidienne entre deux vecteurs position.
    static float distance(Vector3D &v, Vector3D &t);

    /// Addition vectorielle.
    Vector3D operator+(const Vector3D &v) const;
    /// Soustraction vectorielle.
    Vector3D operator-(const Vector3D &v) const;

    /// Normalise le vecteur en place et le retourne.
    Vector3D normalize();
    /// Produit scalaire avec un autre vecteur.
    float dot(const Vector3D &v) const;
    /// Produit vectoriel avec un autre vecteur.
    Vector3D cross(const Vector3D &v) const;

    /// Norme euclidienne.
    float GetNorm();
    /// Norme quadratique (norme au carré).
    float GetSquareNorm();

public:
    /// Composantes cartésiennes du vecteur.
    float x, y, z;
};

#endif //MOTEURPHYSIQUE_VECTOR3D_H
