#include "Quaternion.h"
#include "QuaternionTest.h"

#include <cassert>
#include <iostream>
#include <ostream>

const float Epsilon = 1e-6f;
const float PI = 3.1415926535f;

bool AreEqual(const Quaternion& a, const Quaternion& b) {

	return std::abs(a.w - b.w) < Epsilon
		&& std::abs(a.x - b.x) < Epsilon
		&& std::abs(a.y - b.y) < Epsilon
		&& std::abs(a.z - b.z) < Epsilon;
}

bool AreEqual(const Vector3D& a, const Vector3D& b) {
	return std::abs(a.x - b.x) < Epsilon
	   && std::abs(a.y - b.y) < Epsilon
	   && std::abs(a.z - b.z) < Epsilon;
}

// Remplacer par vos macros de test
#define ASSERT_TRUE(condition) assert(condition)
#define ASSERT_QUATERNION_EQUAL(q1, q2) assert(AreEqual(q1, q2))
#define ASSERT_FLOAT_EQUAL(f1, f2) assert(std::abs((f1) - (f2)) < Epsilon)

const Quaternion Identity(1.f, 0.f, 0.f, 0.f);

const Quaternion Zero(0.f, 0.f, 0.f, 0.f);

const Quaternion Seq( 2,  6,  -1,  4);

void QuaternionTest::RunAllTests() {
	Test_Default_Constructor();
	Test_Constructor();
	Test_CreateFromAxisAngle();
	Test_Multiply();
	Test_CreateFromEulerXYZ();
	Test_Identity();
	Test_Length();
	Test_Normalize();
	Test_Normalized();
	Test_Conjugate();
	Test_Inverse();
	Test_Rotate();
	Test_Slerp();

	std::cout << "Tests Quaternion réussi" << '\n';
}

void QuaternionTest::Test_Default_Constructor() {
	const Quaternion q;

	ASSERT_QUATERNION_EQUAL(q, Zero);
}

void QuaternionTest::Test_Constructor() {
	const Quaternion q(2, 6, -1, 4);

	ASSERT_QUATERNION_EQUAL(q, Seq);
}

void QuaternionTest::Test_CreateFromAxisAngle() {
	Vector3D axis = Vector3D(6.f, -1.f, 4.f);
	axis.normalize();

	float half = 0.5f * 2.f;
	float s  = std::sin(half);

	Quaternion expected(std::cos(half), axis.x*s, axis.y*s, axis.z*s);

	ASSERT_QUATERNION_EQUAL(expected, Quaternion::FromAxisAngle(axis, 2));
}

void QuaternionTest::Test_CreateFromEulerXYZ() {
    // Test 1: Pas de rotation
    Quaternion q_zero = Quaternion::FromEulerXYZ(0.f, 0.f, 0.f);
    ASSERT_QUATERNION_EQUAL(q_zero, Identity);

    // Test 2: Rotation de 90 degrés sur X
    Quaternion q_x90 = Quaternion::FromEulerXYZ(PI / 2.f, 0.f, 0.f);
    Quaternion expected_x90 = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), PI / 2.f);
    ASSERT_QUATERNION_EQUAL(q_x90, expected_x90);

    // Test 3: Rotation de 90 degrés sur Y
    Quaternion q_y90 = Quaternion::FromEulerXYZ(0.f, PI / 2.f, 0.f);
    Quaternion expected_y90 = Quaternion::FromAxisAngle(Vector3D(0, 1, 0), PI / 2.f);
    ASSERT_QUATERNION_EQUAL(q_y90, expected_y90);

    // Test 4: Rotations combinées (X puis Y)
    // qx = (c, s, 0, 0) avec c=cos(pi/4), s=sin(pi/4)
    // qy = (c, 0, s, 0)
    // L'ordre est qz * (qy * qx)
    Quaternion q_xy = Quaternion::FromEulerXYZ(PI / 2.f, PI / 2.f, 0.f);

    // Résultat de qy * qx
    // w = c*c - 0*s - s*0 - 0*0 = c*c = 0.5
    // x = c*s + 0*c + s*0 - 0*0 = c*s = 0.5
    // y = c*0 - 0*0 + s*c + 0*s = s*c = 0.5
    // z = c*0 + 0*0 - s*s + 0*c = -s*s = -0.5
    Quaternion expected_xy(0.5f, 0.5f, 0.5f, -0.5f);
    ASSERT_QUATERNION_EQUAL(q_xy, expected_xy);
}

void QuaternionTest::Test_Identity() {
    ASSERT_QUATERNION_EQUAL(Quaternion::Identity(), Identity);
}

void QuaternionTest::Test_Length() {
    // Longueur de Zéro
    ASSERT_FLOAT_EQUAL(Zero.length(), 0.f);

    // Longueur de l'Identité
    ASSERT_FLOAT_EQUAL(Identity.length(), 1.f);

    // Longueur de Seq (2, 6, -1, 4)
    // sqrt(2*2 + 6*6 + (-1)*(-1) + 4*4) = sqrt(4 + 36 + 1 + 16) = sqrt(57)
    ASSERT_FLOAT_EQUAL(Seq.length(), std::sqrt(57.f));
}

void QuaternionTest::Test_Normalize() {
    // Normaliser Zéro (cas spécial, devient Identité)
    Quaternion qz = Zero;
    qz.normalize();
    ASSERT_QUATERNION_EQUAL(qz, Identity);

    // Normaliser Seq
    Quaternion q_seq = Seq;
    q_seq.normalize();
    ASSERT_FLOAT_EQUAL(q_seq.length(), 1.f); // Doit avoir une longueur de 1

    float inv_len = 1.f / std::sqrt(57.f);
    Quaternion expected(2.f*inv_len, 6.f*inv_len, -1.f*inv_len, 4.f*inv_len);
    ASSERT_QUATERNION_EQUAL(q_seq, expected);
}

void QuaternionTest::Test_Normalized() {
    // Normalized ne doit pas modifier l'original
    Quaternion q_seq = Seq;
    Quaternion q_norm = q_seq.normalized();

    // L'original est inchangé
    ASSERT_QUATERNION_EQUAL(q_seq, Seq);

    // Le nouveau est normalisé
    ASSERT_FLOAT_EQUAL(q_norm.length(), 1.f);
    float inv_len = 1.f / std::sqrt(57.f);
    Quaternion expected(2.f*inv_len, 6.f*inv_len, -1.f*inv_len, 4.f*inv_len);
    ASSERT_QUATERNION_EQUAL(q_norm, expected);
}

void QuaternionTest::Test_Conjugate() {
    // Conjugué de l'Identité
    ASSERT_QUATERNION_EQUAL(Identity.conjugate(), Identity);

    // Conjugué de Seq (2, 6, -1, 4) -> (2, -6, 1, -4)
    Quaternion expected(2.f, -6.f, 1.f, -4.f);
    ASSERT_QUATERNION_EQUAL(Seq.conjugate(), expected);
}

void QuaternionTest::Test_Inverse() {
    // Inverse de Zéro (cas spécial, retourne Zéro)
    ASSERT_QUATERNION_EQUAL(Zero.inverse(), Zero);

    // Inverse de l'Identité
    ASSERT_QUATERNION_EQUAL(Identity.inverse(), Identity);

    // q * q_inv = Identity
    Quaternion q_inv = Seq.inverse();
    Quaternion result = Seq * q_inv;
    ASSERT_QUATERNION_EQUAL(result, Identity);

    // q_inv * q = Identity
    Quaternion result2 = q_inv * Seq;
    ASSERT_QUATERNION_EQUAL(result2, Identity);
}

void QuaternionTest::Test_Multiply() {
    // q * Identity = q
    ASSERT_QUATERNION_EQUAL(Seq * Identity, Seq);

    // Identity * q = q
    ASSERT_QUATERNION_EQUAL(Identity * Seq, Seq);

    // q * Zero = Zero
    ASSERT_QUATERNION_EQUAL(Seq * Zero, Zero);

    // Test: i * j = k
    Quaternion i(0, 1, 0, 0);
    Quaternion j(0, 0, 1, 0);
    Quaternion k(0, 0, 0, 1);
    ASSERT_QUATERNION_EQUAL(i * j, k);

    // Test: 90-deg X rot * 90-deg X rot = 180-deg X rot
    Quaternion q_x90 = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), PI / 2.f);
    Quaternion q_x180 = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), PI);
    ASSERT_QUATERNION_EQUAL(q_x90 * q_x90, q_x180);
}

void QuaternionTest::Test_Rotate() {
    Vector3D v_x(1, 0, 0);
    Vector3D v_y(0, 1, 0);
    Vector3D v_z(0, 0, 1);

    // Test 1: Rotation par Identité
    ASSERT_TRUE(AreEqual(Identity.rotate(v_x), v_x));
    ASSERT_TRUE(AreEqual(Identity.rotate(v_y), v_y));

    // Test 2: Rotation de 90 deg sur X : (0, 1, 0) -> (0, 0, 1)
    Quaternion q_x90 = Quaternion::FromAxisAngle(v_x, PI / 2.f);
    Vector3D r_x1 = q_x90.rotate(v_y);
    ASSERT_TRUE(AreEqual(r_x1, v_z));

    // Test 3: Rotation de 90 deg sur Y : (0, 0, 1) -> (1, 0, 0)
    Quaternion q_y90 = Quaternion::FromAxisAngle(v_y, PI / 2.f);
    Vector3D r_y1 = q_y90.rotate(v_z);
    ASSERT_TRUE(AreEqual(r_y1, v_x));

    // Test 4: Rotation de 90 deg sur Z : (1, 0, 0) -> (0, 1, 0)
    Quaternion q_z90 = Quaternion::FromAxisAngle(v_z, PI / 2.f);
    Vector3D r_z1 = q_z90.rotate(v_x);
    ASSERT_TRUE(AreEqual(r_z1, v_y));
}

void QuaternionTest::Test_Slerp() {
    Quaternion q_x90 = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), PI / 2.f);
    Quaternion q_x45 = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), PI / 4.f);

    // Test 1: t = 0
    Quaternion r0 = Quaternion::Slerp(Identity, q_x90, 0.f);
    ASSERT_QUATERNION_EQUAL(r0, Identity);

    // Test 2: t = 1
    Quaternion r1 = Quaternion::Slerp(Identity, q_x90, 1.f);
    ASSERT_QUATERNION_EQUAL(r1, q_x90);

    // Test 3: t = 0.5 (chemin le plus court)
    Quaternion r_half = Quaternion::Slerp(Identity, q_x90, 0.5f);
    ASSERT_QUATERNION_EQUAL(r_half, q_x45);

    // Test 4: Chemin le plus long (dot < 0)
    // Slerp de q à -q doit prendre le chemin le plus court (comme slerp de q à q)
    Quaternion q_neg = Quaternion(-q_x90.w, -q_x90.x, -q_x90.y, -q_x90.z);
    Quaternion r_neg = Quaternion::Slerp(q_x90, q_neg, 0.5f);
    // Le Slerp gère `dot < 0` en inversant `bb` et `dot`, donc il interpole
    // de q_x90 à q_x90, ce qui devrait donner q_x90.
    ASSERT_QUATERNION_EQUAL(r_neg, q_x90.normalized()); // S'assure qu'il est bien normalisé

    // Test 5: Interpolation linéaire (Nlerp) (quaternions très proches)
    Quaternion q_close = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), 1e-7f);
    Quaternion r_nlerp = Quaternion::Slerp(Identity, q_close, 0.5f);
    Quaternion expected_nlerp = Quaternion::FromAxisAngle(Vector3D(1, 0, 0), 0.5f * 1e-7f);
    ASSERT_QUATERNION_EQUAL(r_nlerp, expected_nlerp);
}
