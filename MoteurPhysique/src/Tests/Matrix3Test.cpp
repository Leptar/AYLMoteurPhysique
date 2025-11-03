#include "Matrix3Test.h"

#include <cassert>
#include <iostream>

const float PI_F = 3.1415926535f;

#define ASSERT_TRUE(condition) assert(condition)
#define ASSERT_MATRIX_EQUAL(m1, m2) assert(Matrix3::AreEqual(m1, m2))
#define ASSERT_VECTOR_EQUAL(v1, v2) assert(Matrix3::AreEqual(v1, v2))

void Matrix3Test::TestAddition() {
	Result = Matrix3(2.f, 6.f, 10.f,
					  6.f, 10.f, 14.f,
					  10.f, 14.f, 18.f);

	Matrix3 FindMatrix = AMatrix + BMatrix;
	ASSERT_MATRIX_EQUAL(Result, FindMatrix);

	std::cout << "Matrix3 addition validé" << '\n';
}

void Matrix3Test::TestSubtraction() {
	Result = Matrix3(0.f, -2.f, -4.f,
				      2.f, 0.f, -2.f,
				      4.f, 2.f, 0.f);
	Matrix3 FindMatrix = AMatrix - BMatrix;
	ASSERT_MATRIX_EQUAL(Result, FindMatrix);

	std::cout << "Matrix3 soustraction validé" << '\n';
}

void Matrix3Test::TestMultiplication() {
	Result = Matrix3(14.f, 32.f, 50.f,
					 32.f, 77.f, 122.f,
					 50.f, 122.f, 194.f);

	Matrix3 FindMatrix = AMatrix * BMatrix;
	ASSERT_MATRIX_EQUAL(Result, FindMatrix);

	std::cout << "Matrix3 multiplication validé" << '\n';
}

void Matrix3Test::TestDivision() {
	Result = Matrix3(0.5f, 1.f, 1.5f,
					 2.f, 2.5f, 3.f,
					 3.5f, 4.f, 4.5f);

	Matrix3 FindMatrix = AMatrix/2.f;
	ASSERT_MATRIX_EQUAL(Result, FindMatrix);

	std::cout << "Matrix3 division validé" << '\n';
}

void Matrix3Test::TestTranspose() {
	ASSERT_MATRIX_EQUAL(AMatrix.transpose(), BMatrix);
	std::cout << "Matrix3 transpose validé" << '\n';
}

void Matrix3Test::TestDeterminant() {
	float result = AMatrix.determinant();
	assert(result == 0.f);
	result = CMatrix.determinant();
	assert(result == 188.f);

	std::cout << "Matrix3 inverse validé" << '\n';
}

void Matrix3Test::TestRotationZero() {
	Matrix3 id = Matrix3::Identity();

	Matrix3 mX = Matrix3::RotX(0.0f);
	ASSERT_MATRIX_EQUAL(mX, id);

	Matrix3 mY = Matrix3::RotY(0.0f);
	ASSERT_MATRIX_EQUAL(mY, id);

	Matrix3 mZ = Matrix3::RotZ(0.0f);
	ASSERT_MATRIX_EQUAL(mZ, id);
}

void Matrix3Test::TestRotation_90Degrees() {
	float angle90 = PI_F / 2.0f;


	Matrix3 mX = Matrix3::RotX(angle90);
	Matrix3 expectedX(1, 0, 0,
					  0, 0,-1,
					  0, 1, 0);
	ASSERT_MATRIX_EQUAL(mX, expectedX);


	Matrix3 mY = Matrix3::RotY(angle90);
	Matrix3 expectedY(0, 0, 1,
					  0, 1, 0,
					 -1, 0, 0);
	ASSERT_MATRIX_EQUAL(mY, expectedY);


	Matrix3 mZ = Matrix3::RotZ(angle90);
	Matrix3 expectedZ(0,-1, 0,
					  1, 0, 0,
					  0, 0, 1);
	ASSERT_MATRIX_EQUAL(mZ, expectedZ);
}

void Matrix3Test::TestRotation_Transformation() {

	float angle90 = PI_F / 2.0f;
	Vector3D vX(1.0f, 0.0f, 0.0f);
	Vector3D vY(0.0f, 1.0f, 0.0f);
	Vector3D vZ(0.0f, 0.0f, 1.0f);


	Matrix3 mZ = Matrix3::RotZ(angle90);
	Vector3D resultZ = mZ * vX;
	ASSERT_VECTOR_EQUAL(resultZ, vY);

	Matrix3 mX = Matrix3::RotX(angle90);
	Vector3D resultX = mX * vY;
	ASSERT_VECTOR_EQUAL(resultX, vZ);

	Matrix3 mY = Matrix3::RotY(angle90);
	Vector3D resultY = mY * vZ;
	ASSERT_VECTOR_EQUAL(resultY, vX);
}

void Matrix3Test::Test_AxisAngle_EdgeCases() {
	Matrix3 id = Matrix3::Identity();
	Vector3D arbitraryAxis(1.0f, 2.0f, 3.0f);
	Vector3D zeroAxis(0.0f, 0.0f, 0.0f);

	Matrix3 m1 = Matrix3::AxisAngle(arbitraryAxis, 0.0f);
	ASSERT_MATRIX_EQUAL(m1, id);

	Matrix3 m2 = Matrix3::AxisAngle(zeroAxis, PI_F / 4.0f);
	ASSERT_MATRIX_EQUAL(m2, id);
}

void Matrix3Test::test_axis_angle_equivalence() {
	float angle = 0.785f;
	Vector3D axisX(1.0f, 0.0f, 0.0f);
	Vector3D axisY(0.0f, 1.0f, 0.0f);
	Vector3D axisZ(0.0f, 0.0f, 1.0f);

	Matrix3 mX = Matrix3::AxisAngle(axisX, angle);
	ASSERT_MATRIX_EQUAL(mX, Matrix3::RotX(angle));

	Matrix3 mY = Matrix3::AxisAngle(axisY, angle);
	ASSERT_MATRIX_EQUAL(mY, Matrix3::RotY(angle));

	Matrix3 mZ = Matrix3::AxisAngle(axisZ, angle);
	ASSERT_MATRIX_EQUAL(mZ, Matrix3::RotZ(angle));
}

void Matrix3Test::Test_AxisAngle_Normalization() {
	float angle = PI_F / 6.0f;

	Vector3D axisY_NonUnit(0.0f, 5.0f, 0.0f);

	Matrix3 m1 = Matrix3::AxisAngle(axisY_NonUnit, angle);
	Matrix3 m2 = Matrix3::RotY(angle);

	ASSERT_MATRIX_EQUAL(m1, m2);
}

void Matrix3Test::Test_Property_Orthogonal() {
	Matrix3 id = Matrix3::Identity();

	// Testez avec des angles arbitraires
	float angle1 = 0.45f;
	float angle2 = -1.2f;
	Vector3D axis(0.5f, 0.5f, 0.7f); // Axe non trivial

	Matrix3 matricesToTest[] = {
		Matrix3::RotX(angle1),
		Matrix3::RotY(angle2),
		Matrix3::RotZ(angle1 + angle2),
		Matrix3::AxisAngle(axis, angle1)
	};

	for (const auto& m : matricesToTest) {
		Matrix3 m_T = m.transpose(); // Supposant que vous avez .Transpose()
		Matrix3 result = m_T * m;    // Supposant que vous avez M * M

		// m_T * m doit être l'identité
		ASSERT_MATRIX_EQUAL(result, id);
	}
}

void Matrix3Test::TestInverse() {
	Result = Matrix3::Identity();
	Matrix3 FindMatrix = CMatrix.inverse();

	ASSERT_MATRIX_EQUAL(CMatrix*Result, CMatrix);
	ASSERT_MATRIX_EQUAL(Result, CMatrix*FindMatrix);

	std::cout << "Matrix3 inverse validé" << '\n';
}


void Matrix3Test::RunAllTests() {
	TestAddition();
	TestSubtraction();
	TestMultiplication();
	TestDivision();

	TestTranspose();
	TestDeterminant();
	TestInverse();

	TestRotationZero();
	TestRotation_90Degrees();
	TestRotation_Transformation();
	std::cout << "Matrix3 Rotation via X/Y/Z validé" << '\n';
	Test_AxisAngle_EdgeCases();
	test_axis_angle_equivalence();
	Test_AxisAngle_Normalization();
	Test_Property_Orthogonal();
	std::cout << "Matrix3 Rotation via Axe validé" << '\n';

	std::cout << "Tests validés" << '\n';

}
