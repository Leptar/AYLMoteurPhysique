# Guide des corrections pour les erreurs de compilation

## Résumé
Les erreurs proviennent de l'utilisation de `.transform()` qui n'existe pas dans Matrix3 et Matrix4.
**Solution : Utiliser l'opérateur `*` qui existe déjà dans le code.**

---

## ✅ Correction 1 : CollisionPrimitive.cpp (ligne 36)

### Erreur
```cpp
// INCORRECT - 'transform' n'existe pas
matrix4Instance.transform(vector)
```

### Correction
```cpp
// CORRECT - utiliser l'opérateur *
matrix4Instance * vector
```

---

## ✅ Correction 2 : RigidBodyContact.cpp (lignes 53, 62, 77, 88)

### Erreur
```cpp
// INCORRECT - 'transform' n'existe pas
matrix3Instance.transform(vector)
```

### Correction
```cpp
// CORRECT - utiliser l'opérateur *
matrix3Instance * vector
```

**Exemple concret :**
```cpp
// AVANT
Vector3D transformed = rotationMatrix.transform(contactNormal);

// APRÈS
Vector3D transformed = rotationMatrix * contactNormal;
```

---

## ✅ Correction 3 : RigidBodyContact.cpp (ligne 158)

### Erreur
```
Error C2248 : 'RigidBodyContact::calculateSeparatingVelocity' :
impossible d'accéder à protected membre
```

### Solutions possibles

#### Option A : Rendre la méthode publique
```cpp
// Dans RigidBodyContact.h
class RigidBodyContact {
public:
    float calculateSeparatingVelocity();  // Déplacer de protected à public
    // ...
};
```

#### Option B : Utiliser une méthode publique wrapper
```cpp
// Dans RigidBodyContact.h
class RigidBodyContact {
public:
    float getSeparatingVelocity() {
        return calculateSeparatingVelocity();
    }
protected:
    float calculateSeparatingVelocity();
    // ...
};
```

#### Option C : Vérifier l'appel
Si l'appel se fait à la ligne 158 de RigidBodyContact.cpp, assurez-vous qu'il est fait :
- Soit depuis une méthode membre de RigidBodyContact
- Soit depuis une classe amie (friend class)
- Sinon, l'appel doit être fait via une méthode publique

---

## Opérateurs disponibles

### Matrix3
- ✅ `Matrix3 * Vector3D` → transforme un vecteur (rotation 3D)
- ✅ `Matrix3 * Matrix3` → multiplication de matrices

### Matrix4
- ✅ `Matrix4 * Vector3D` → transforme un vecteur (rotation + translation)
- ✅ `Matrix4 * Matrix4` → multiplication de matrices

---

## Exemple complet de transformation

```cpp
// Transformation avec Matrix3 (rotation uniquement)
Matrix3 rotation = Matrix3::RotZ(3.14f / 4.0f);
Vector3D direction(1, 0, 0);
Vector3D rotatedDirection = rotation * direction;

// Transformation avec Matrix4 (rotation + translation)
Matrix4 transform;
transform.SetRotation(rotation);
transform.SetPosition(Vector3D(10, 20, 30));
Vector3D point(0, 0, 0);
Vector3D transformedPoint = transform * point;
```
