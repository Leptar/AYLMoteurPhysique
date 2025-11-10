#include "Matrix4.h"
#include "Matrix4Test.h"
#include <cassert>
#include <iostream>

const float Epsilon = 1e-6f;

// Fonction de comparaison pour Matrix4
bool AreEqual(const Matrix4& a, const Matrix4& b) {
	for (int i = 0; i < 16; ++i) {
		if (std::abs(a.m[i] - b.m[i]) > Epsilon) {
			return false;
		}
	}
	return true;
}

// Remplacer par vos macros de test
#define ASSERT_FALSE(condition) assert(!(condition))
#define ASSERT_MATRIX_EQUAL(m1, m2) assert(AreEqual(m1, m2))
#define ASSERT_FLOAT_EQUAL(f1, f2) assert(std::abs((f1) - (f2)) < Epsilon)

const Matrix4 Identity(1.f, 0, 0, 0,
					   0, 1.f, 0, 0,
					   0, 0, 1.f, 0,
					   0, 0, 0, 1.f);

const Matrix4 Zero(0, 0, 0, 0,
				   0, 0, 0, 0,
				   0, 0, 0, 0,
				   0, 0, 0, 0);

const Matrix4 Seq( 1,  2,  3,  4,
				   5,  6,  7,  8,
				   9, 10, 11, 12,
				  13, 14, 15, 16);

void Test_Constructor_Default() {
	Matrix4 m;

	ASSERT_MATRIX_EQUAL(m, Zero);
}

void Test_Constructor_Diagonal() {
	// Test 1: Identité
	Matrix4 m_id(1.0f);
	ASSERT_MATRIX_EQUAL(m_id, Identity);

	// Test 2: Matrice de scaling
	Matrix4 m_scale(5.0f);
	Matrix4 expected(5, 0, 0, 0,
					 0, 5, 0, 0,
					 0, 0, 5, 0,
					 0, 0, 0, 5);
	ASSERT_MATRIX_EQUAL(m_scale, expected);
}

void Test_Constructor_Full() {
	Matrix4 m = Seq;
	ASSERT_FLOAT_EQUAL(m.m[0], 1.f);
	ASSERT_FLOAT_EQUAL(m.m[1], 2.f);
	ASSERT_FLOAT_EQUAL(m.m[4], 5.f);
	ASSERT_FLOAT_EQUAL(m.m[15], 16.f);
}

void Test_Method_Identity() {
	Matrix4 m_junk = Seq;
	Matrix4 m_id = m_junk.Identity();
	ASSERT_MATRIX_EQUAL(m_id, Identity);

	ASSERT_MATRIX_EQUAL(m_junk.Identity(), Matrix4(1.0f));
}

void Test_Operators_AddSubtract() {
	Matrix4 A(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4);
	Matrix4 B(5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8);

	Matrix4 expected_add(6, 7, 8, 9, 7, 8, 9, 10, 8, 9, 10, 11, 9, 10, 11, 12);
	Matrix4 expected_sub(-4,-3,-2,-1,-5,-4,-3,-2,-6,-5,-4,-3,-7,-6,-5,-4);

	ASSERT_MATRIX_EQUAL(A + B, expected_add);
	ASSERT_MATRIX_EQUAL(A - B, expected_sub);
	ASSERT_MATRIX_EQUAL(A - A, Zero);
	ASSERT_MATRIX_EQUAL(A + Zero, A);
}

void Test_Operator_DivideScalar() {
	Matrix4 m(10, 20, -30, 0,
			  0,  10,  0,  0,
			  0,   0, 10,  0,
			  0,   0,  0, 10);

	Matrix4 expected(1, 2, -3, 0,
					 0, 1,  0, 0,
					 0, 0,  1, 0,
					 0, 0,  0, 1);

	ASSERT_MATRIX_EQUAL(m / 10.0f, expected);

}

void Test_Operator_Multiply_Identity() {
	Matrix4 m = Seq;
	ASSERT_MATRIX_EQUAL(m * Identity, m);
	ASSERT_MATRIX_EQUAL(Identity * m, m);
}

void Test_Operator_Multiply_KnownValue() {
	// Matrice de translation
	Matrix4 trans(1, 0, 0, 5,
				  0, 1, 0, 6,
				  0, 0, 1, 7,
				  0, 0, 0, 1);

	// Matrice de scaling
	Matrix4 scale(2, 0, 0, 0,
				  0, 3, 0, 0,
				  0, 0, 4, 0,
				  0, 0, 0, 1);

	// Résultat attendu : scale * trans
	Matrix4 expected_S_T(2, 0, 0, 10,
						 0, 3, 0, 18,
						 0, 0, 4, 28,
						 0, 0, 0, 1);

	// Résultat attendu : trans * scale
	Matrix4 expected_T_S(2, 0, 0, 5,
						 0, 3, 0, 6,
						 0, 0, 4, 7,
						 0, 0, 0, 1);

	ASSERT_MATRIX_EQUAL(scale * trans, expected_S_T);
	ASSERT_MATRIX_EQUAL(trans * scale, expected_T_S);
	ASSERT_FALSE(AreEqual(scale * trans, trans * scale));
}

void Test_Method_Transpose() {
	Matrix4 m = Seq;

	Matrix4 expected(1, 5, 9, 13,
					 2, 6, 10, 14,
					 3, 7, 11, 15,
					 4, 8, 12, 16);

	ASSERT_MATRIX_EQUAL(m.transpose(), expected);

	ASSERT_MATRIX_EQUAL(m.transpose().transpose(), m);

	Matrix4 A = Matrix4(1, 0, 0, 5, 0, 1, 0, 6, 0, 0, 1, 7, 0, 0, 0, 1);
	Matrix4 B = Matrix4(2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 1);

	Matrix4 AB_T = (A * B).transpose();
	Matrix4 B_T_A_T = B.transpose() * A.transpose();
	ASSERT_MATRIX_EQUAL(AB_T, B_T_A_T);
}

void Test_Method_Determinant() {

	ASSERT_FLOAT_EQUAL(Identity.determinant(), 1.0f);

	Matrix4 scale(2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 5);
	ASSERT_FLOAT_EQUAL(scale.determinant(), 120.0f);


	Matrix4 singular(1, 2, 3, 4,
					 1, 2, 3, 4,
					 5, 6, 7, 8,
					 9, 10, 11, 12);
	ASSERT_FLOAT_EQUAL(singular.determinant(), 0.0f);

	Matrix4 A = Matrix4(1, 0, 0, 5, 0, 1, 0, 6, 0, 0, 1, 7, 0, 0, 0, 1); // det(A) = 1
	Matrix4 B = Matrix4(2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 1); // det(B) = 2*3*4 = 24
	ASSERT_FLOAT_EQUAL((A * B).determinant(), A.determinant() * B.determinant());

	ASSERT_FLOAT_EQUAL(A.transpose().determinant(), A.determinant());
}

void Test_Method_Inverse() {

	ASSERT_MATRIX_EQUAL(Identity.inverse(), Identity);

	Matrix4 scale(2, 0, 0, 0,
				  0, 4, 0, 0,
				  0, 0, 5, 0,
				  0, 0, 0, 1);
	Matrix4 expected_inv_scale(1.f/2.f, 0, 0, 0,
							   0, 1.f/4.f, 0, 0,
							   0, 0, 1.f/5.f, 0,
							   0, 0, 0, 1);
	ASSERT_MATRIX_EQUAL(scale.inverse(), expected_inv_scale);

	Matrix4 trans(1, 0, 0, 10,
				  0, 1, 0, -5,
				  0, 0, 1, 20,
				  0, 0, 0, 1);
	Matrix4 inv_trans = trans.inverse();

	Matrix4 expected_inv_trans(1, 0, 0, -10,
							   0, 1, 0, 5,
							   0, 0, 1, -20,
							   0, 0, 0, 1);
	ASSERT_MATRIX_EQUAL(inv_trans, expected_inv_trans);

	ASSERT_MATRIX_EQUAL(trans * inv_trans, Identity);
	ASSERT_MATRIX_EQUAL(inv_trans * trans, Identity);

	Matrix4 singular(1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4);

	Matrix4 inv = singular.inverse();
	ASSERT_MATRIX_EQUAL(inv, Identity);
}

void RunAllTestsMatrix4() {
	Test_Constructor_Default();
	Test_Constructor_Diagonal();
	Test_Method_Identity();
	Test_Operator_DivideScalar();
	Test_Operator_Multiply_Identity();
	Test_Operator_Multiply_KnownValue();
	Test_Operators_AddSubtract();
	Test_Method_Transpose();
	Test_Method_Determinant();
	Test_Method_Inverse();

	std::cout << "Matrix4 Test validé" << std::endl;
}
